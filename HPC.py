import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim
from torch.utils.data import DataLoader, TensorDataset
from tqdm import tqdm


class Conv_AE(nn.Module):
	def __init__(self, n_hidden=100, hidden_regularization=False):
		'''
		Convolutional autoencoder in PyTorch, prepared to process images of shape (360,640,3). A sparsity constraint can be added to the middle layer.

		Args:
			n_hidden (int; default=100): number of hidden units in the middle layer.
		'''
		super().__init__()

		self.hidden_regularization = hidden_regularization

		self.n_hidden = n_hidden
		self.dim1, self.dim2 = 45, 80  # Adjusted dimensions after encoder

		# Encoder
		self.conv1 = nn.Conv2d(3, 16, kernel_size=4, stride=2, padding=1)
		self.conv2 = nn.Conv2d(16, 32, kernel_size=4, stride=2, padding=1)
		self.conv3 = nn.Conv2d(32, 64, kernel_size=4, stride=2, padding=1)
		self.fc1 = nn.Linear(64 * self.dim1 * self.dim2, n_hidden)

		# Decoder
		self.fc2 = nn.Linear(n_hidden, 64 * self.dim1 * self.dim2)
		self.conv4 = nn.ConvTranspose2d(64, 32, kernel_size=4, stride=2, padding=1, output_padding=1)
		self.conv5 = nn.ConvTranspose2d(32, 16, kernel_size=4, stride=2, padding=1, output_padding=0)
		self.conv6 = nn.ConvTranspose2d(16, 3, kernel_size=4, stride=2, padding=1, output_padding=0)

	def encoder(self, x):
        # Encoder
        x = F.relu(self.conv1(x))
        x = F.relu(self.conv2(x))
        x = F.relu(self.conv3(x))  
        x = x.view(-1, 64 * self.dim1 * self.dim2)  
        x = F.relu(self.fc1(x))
        return x

	def decoder(self, x):
		# Decoder
		x = F.relu(self.fc2(x)) 
		x = x.view(-1, 64, self.dim1, self.dim2) 
		x = F.relu(self.conv4(x)) 
		x = F.relu(self.conv5(x))  
		x = torch.sigmoid(self.conv6(x))  
		
		# Crop the image to match the target size (360, 640)
		x = F.pad(x, (-2, -2, -2, -2))  # Cropping 2 pixels from all sides
		
		return x

	def forward(self, x):
		h = self.encoder(x)
		out = self.decoder(h)
		return out, h

	def backward(self, optimizer, criterion, x, y_true, alpha=0, beta=0):        
		optimizer.zero_grad()

		y_pred, hidden = self.forward(x)

		recon_loss = criterion(y_pred, y_true)

		# Whitening loss (soft batch whitening).
		hidden_reg_loss = 0
		sparsity_loss = 0
		batch_size, hidden_dim = hidden.shape
		if self.hidden_regularization:

			# SSCP matrix
			#M = torch.mm(hidden.t(), hidden)

			# Covariance matrix
			hidden_centered = hidden - torch.mean(hidden, dim=0, keepdim=True)
			M = torch.mm(hidden_centered.t(), hidden_centered) / (batch_size-1)

			I = torch.eye(hidden_dim, device='cuda')
			lambda_ = 1 #0.1
			C = lambda_*I - M   # whitening --> generates spatial tuning
			hidden_reg_loss = alpha * torch.norm(C) / (batch_size*hidden_dim)
		
			sparsity_loss = beta * torch.norm(hidden) / (batch_size*hidden_dim)  # L1 regularization

		loss = recon_loss + hidden_reg_loss + sparsity_loss
		loss.backward()

		optimizer.step()

		return recon_loss.item()


def create_dataloader(dataset, batch_size=256, reshuffle_after_epoch=True):
	'''
	Creates a DataLoader for Pytorch to train the autoencoder with the image data converted to a tensor.

	Args:
		dataset (4D numpy array): image dataset with shape (n_samples, n_channels, n_pixels_height, n_pixels_width).
		batch_size (int; default=32): the size of the batch updates for the autoencoder training.

	Returns:
		DataLoader (Pytorch DataLoader): dataloader that is ready to be used for training an autoencoder.
	'''
	if dataset.shape[-1] == 3:
		dataset = np.transpose(dataset, (0,3,1,2))
	tensor_dataset = TensorDataset(torch.from_numpy(dataset).float(), torch.from_numpy(dataset).float())
	return DataLoader(tensor_dataset, batch_size=batch_size, shuffle=reshuffle_after_epoch)


def train_autoencoder(model, train_loader, dataset=[], num_epochs=1000, learning_rate=1e-4, alpha=2e3, beta=0, L2_weight_decay=0):
					  #L1_lambda=0,  soft_sparsity_weight=0):
	'''
	TO DO.
	'''
	optimizer = optim.Adam(model.parameters(), lr=learning_rate, weight_decay=L2_weight_decay)
	criterion = nn.MSELoss()

	model = model.to('cuda')

	history = []
	embeddings = []
	if len(dataset) > 0:
		embeddings = [ get_latent_vectors(dataset=dataset, model=model) ]
	for epoch in range(num_epochs):
		running_loss = 0.
		with tqdm(total=len(train_loader)) as pbar:
			for i, data in enumerate(train_loader, 0):
				inputs, _ = data
				inputs = inputs.to('cuda')

				loss = model.backward(optimizer=optimizer, criterion=criterion, x=inputs, y_true=inputs, alpha=alpha, beta=beta)
				running_loss += loss

				pbar.update(1)
				pbar.set_description(f"Epoch {epoch+1}/{num_epochs}, Loss: {running_loss/len(train_loader):.4f}")

		history.append(running_loss/len(train_loader))

		if len(dataset) > 0:
			embeddings.append( get_latent_vectors(dataset=dataset, model=model) )

	embeddings = np.array(embeddings)

	return history, embeddings


def predict(image, model):
	'''
	Returns the output of model(image), and reshapes it to be compatible with plotting funtions such as plt.imshow().

	Args:
		image (3D numpy array): sample image with shape (n_channels, n_pixels_height, n_pixels_width).
		model (Pytorch Module): convolutional autoencoder that is prepared to process images such as 'image'.

	Returns:
		output_img (3D numpy array): output image with shape (n_pixels_height, n_pixels_width, n_channels)
	'''
	if image.shape[-1] <= 4:
		image = np.transpose(image, (2,0,1))
	n_channels, n_pixels_height, n_pixels_width = image.shape
	image = np.reshape(image, (1, n_channels, n_pixels_height, n_pixels_width))
	image = torch.from_numpy(image).float().to(next(model.parameters()).device)
	output_img = model(image)[0].detach().cpu().numpy()
	output_img = np.reshape(output_img, (n_channels, n_pixels_height, n_pixels_width))
	output_img = np.transpose(output_img, (1,2,0))
	return output_img


def get_latent_vectors(dataset, model, batch_size=256):
	'''
	Returns the latent activation vectors of the autoencoder model after passing all the images in the dataset.

	Args:
		dataset (numpy array): image dataset with shape 
		model (Pytorch Module): convolutional autoencoder that is prepared to process the images in dataset.

	Returns:
		latent_vectors (2D numpy array): latent activation vectors, matrix with shape (n_samples, n_hidden), where n_hidden is the number of units in the hidden layer.
	'''
	if dataset.shape[-1] <= 4:
		dataset = np.transpose(dataset, (0,3,1,2))
	tensor_dataset = TensorDataset(torch.from_numpy(dataset).float(), torch.from_numpy(dataset).float())
	data_loader = DataLoader(tensor_dataset, batch_size=batch_size, shuffle=False)
	model.eval()
	latent_vectors = []
	with torch.no_grad():
		for batch in data_loader:
			inputs, _ = batch
			latent = model(inputs.to('cuda'))[1]
			latent_vectors.append(latent.cpu().numpy())
	latent_vectors = np.concatenate(latent_vectors)
	return latent_vectors


