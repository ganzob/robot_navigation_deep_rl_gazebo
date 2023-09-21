from keras.models import load_model,Model
from keras.layers import Dense,Input

inp=Input(shape=(7,))
x=Dense(50,activation='sigmoid')(inp)
x=Dense(20,activation='sigmoid')(x)
x=Dense(10,activation='sigmoid')(x)
out=Dense(6,activation='linear')(x)

model=Model(input=inp,output=out)
model.compile(loss='mse',optimizer='adam')
model.load_weights('model_1_weights.h5')
model.save('jetson_model_1.h5')

model.load_weights('model_2_weights.h5')
model.save('jetson_model_2.h5')



