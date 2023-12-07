from keras.models import Sequential
from keras.layers import Dense
import numpy as np
from matplotlib import pyplot as plt

'''
Script for training a map-specific traffic predictor ANN
'''

# Loading training data
X_train = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\classes\X_train.npy').T
y_train = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\classes\Y_train.npy').T
X_test = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\classes\X_test.npy').T
y_test = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\classes\Y_test.npy').T
X_val = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\classes\X_val.npy').T
y_val = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\classes\Y_val.npy').T

# Define the number of input and output parameters
n_inputs = 646
n_outputs = 646

# Define the number of neurons in each hidden layer
n_hidden_1 = 32
n_hidden_2 = 32
n_hidden_3 = 32

# Define the model architecture
model = Sequential()
model.add(Dense(n_hidden_1, input_shape=(646,), input_dim=n_inputs, activation='relu'))
model.add(Dense(n_hidden_2, activation='relu'))
model.add(Dense(n_hidden_3, activation='relu'))
model.add(Dense(n_outputs, activation='linear'))

# Compile the model
model.compile(loss='mse', optimizer='adam', metrics=['accuracy'])

# Train the model
history = model.fit(X_train, y_train, validation_data=(X_val, y_val), epochs=700, batch_size=32)
fig1 = plt.figure()

# Accuracy plot
plt.plot(history.history['accuracy'])
plt.plot(history.history['val_accuracy'])
plt.title('model accuracy')
plt.ylabel('accuracy')
plt.xlabel('epoch')
plt.legend(['train', 'val'], loc='upper left')

fig2 = plt.figure()

# Loss plot
plt.plot(history.history['loss'])
plt.plot(history.history['val_loss'])
plt.title('model loss')
plt.ylabel('loss')
plt.xlabel('epoch')
plt.legend(['train', 'val'], loc='upper left')
plt.savefig('model_loss.jpg')
plt.show()

# Evaluate the model on test data
score, acc = model.evaluate(X_test, y_test)

print(score)
print(acc)

model.save('path/to/location')