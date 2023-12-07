import numpy as np

'''
Script for the assembly of the training data from individually processed traffic log files
'''

x_train_1 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\X_train_2.npy')
x_train_2 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\X_train_4_200.npy')
x_train_3 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\X_train_8_300.npy')
x_train_4 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\X_train_1.npy')
x_train_5 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\X_train_5_200.npy')
x_train_6 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\X_train_9_300.npy')
x_train_7 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\X_train_3.npy')
x_train_8 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\X_train_6_200.npy')
x_train_9 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\X_train_7_300.npy')
y_train_1 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\Y_train_2.npy')
y_train_2 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\Y_train_4_200.npy')
y_train_3 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\Y_train_8_300.npy')
y_train_4 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\Y_train_1.npy')
y_train_5 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\Y_train_5_200.npy')
y_train_6 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\Y_train_9_300.npy')
y_train_7 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\Y_train_3.npy')
y_train_8 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\Y_train_6_200.npy')
y_train_9 = np.load(r'C:\Users\User\PycharmProjects\CarlaPlanner\Y_train_7_300.npy')

X_train = np.concatenate((x_train_1, x_train_2), axis=1)
X_train = np.concatenate((X_train, x_train_3), axis=1)
X_train = np.concatenate((X_train, x_train_4), axis=1)
X_train = np.concatenate((X_train, x_train_8), axis=1)
Y_train = np.concatenate((y_train_1, y_train_2), axis=1)
Y_train = np.concatenate((Y_train, y_train_3), axis=1)
Y_train = np.concatenate((Y_train, y_train_4), axis=1)
Y_train = np.concatenate((Y_train, y_train_8), axis=1)

X_val = np.concatenate((x_train_5, x_train_6), axis=1)
Y_val = np.concatenate((y_train_5, y_train_6), axis=1)

X_test = np.concatenate((x_train_7, x_train_9), axis=1)
Y_test = np.concatenate((y_train_7, y_train_9), axis=1)

np.save('X_train', X_train)
np.save('Y_train', Y_train)
np.save('X_val', X_val)
np.save('Y_val', Y_val)
np.save('X_test', X_test)
np.save('Y_test', Y_test)
