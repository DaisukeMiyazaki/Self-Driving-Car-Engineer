import matplotlib.pyplot as plt
import numpy as np
import cv2
from keras.models import load_model
from PIL import Image
from keras.applications import ResNet50
from keras.utils import plot_model
from keras.preprocessing.image import load_img,img_to_array, array_to_image

from keras import backend as K
NUM = 1

if __name__ == '__main__':
    
    IMG_HEIGHT = 96
    IMG_WIDTH = 71
    
    #model_1 = ResNet50()
    model_path = '/home/student/CarND-Capstone/train_nn/tl_classifier_keras.h5'
    img_path = '/home/student/CarND-Capstone/train_nn/data/GREEN_train/image_1.png'
    img = image.load_img(img_path,(IMG_HEIGHT,IMG_WIDTH))
    img = img.img_to_array(img)
    img = np.expand_dims(img,axis=0)
    img = preprpcess_input(img)
    
    model = load_model(model_path)
    model.summary()
    """
    
    image = cv2.imread(img_path)
    image = np.reshape(image,(1,600,800,3))
    """
    
    """
    #image = cv2.resize(image,(800,600))
    y = model.predict(image)
    print type(y)
    print y.shape
    """
    
   
    # model visualization
    #plot_model(model,to_file='/home/student/CarND-Capstone/train_nn/model.png',show_shapes='True')
    
    
    
    #weights_1 = model_1.get_layer("conv1").get_weights()[0]
    #Lyr_obj = model.get_layer("conv2d_2").output
    #Lyr_obj_1 = model.get_layer("conv2d_2").output_shape
    #Lyr_obj_2 = model.get_layer("conv2d_2").get_output_shape(0)
    
    print "[statsus] Extaract input imgae features..."
    layers = model.layers[:] 
    layer_outputs = [layer.output for layer in layers]
    activation_model = models.Model(inputs=model.input,outputs=layer_outputs)
    # activation_model.summary()
    activations = activation_model.predict(image)
    
    conv_and_pool_activations =[]
    for layer,activation in zip(layers,activations):
        is_pooling_layer = isinstance(layer,MaxPooling2D)
        is_convolution_layer = isinstance(layer,Convlution2D)
        if is_pooling_layer or is_convlution_layer:
            conv_and_pool_activations.append([layer.name,activation])
    
    print "[status] Generatin heatmaps..."
    os.makedirs(args.directory, exist_ok=True)
    for i, (name,activation) in enumerate(conv_and_pool_activations):
        print "[Status] Processing %s layer..." % name
        n_imgs = activation.shape[3]
        n_cols = math.ceil(math.sqrt(n_imgs))
        n_rows = math.floor(n_imgs / n_cols)
        screens = []
        
        for y in range(0, n_rows):
            rows = []
            for x in range(0, n_cols):
                if j < n_imgs:
                    featuremap_img = activation[0,:,:,j]
                    rows.append(featuremap_img)
                else:
                    rows.append(np.zeros())
            screens.append(np.concatenate(rows,axis=1))
        screens = np.concatenate(screens, axis= 0)
        plt.figure()
        sns.heatmap(screens,xticklabels=False, yticklabel=False)
        save_name = "%s.png" % name
        save_path = os.path.join(args.directory, save_name)
        plt.savefig(save_path)
        plt.close
    print "[Status] Generating heatmap has finished ... "
    
    #print Lyr_obj
    
    
    
    """
    #print [i for i in test]
    print [len(i) for i in test]
    tns_1,tns_2,tns_3,tns_4 = weights.shape
   
    kernel_time = (IMG_WIDTH / tns_1) * (IMG_HEIGHT // tns_2)
    # image as kernels combined
    result = Image.new("RGB",(IMG_WIDTH,IMG_HEIGHT))
    for i in range(tns_4):
        w = weights[:,:,:,i].copy()
        m = w.min()
        M = w.max()
        w = (w-m)/(M-m)
        w *= 255
        IMAGE = Image.fromarray(w.astype("uint8"),mode = "RGB")
        COPY = np.array(IMAGE.copy())
        RESULT = result.copy()
        for j in range(kernel_time):
            RESULT.paste(IMAGE,(tns_1*j % IMG_WIDTH, (j//(IMG_WIDTH / tns_1) * tns_2)))
        RESULT = np.array(RESULT)
        #print type(RESULT) 
        visual_path = '/home/student/CarND-Capstone/train_nn/nn_visual/'
        visual_path_COPY = '/home/student/CarND-Capstone/train_nn/nn_visual/COPY/'
        img = 'image_{}.png'
        img_COPY = 'image_COPY_{}.png'
        path = visual_path + img.format(NUM)
        path_COPY = visual_path_COPY + img_COPY.format(NUM)
        NUM += 1
        #IMAGE = np.ndarray(result)
        cv2.imwrite(path,RESULT)
        cv2.imwrite(path_COPY,cv2.resize(COPY,(IMG_WIDTH,IMG_HEIGHT)))
        
    """
    #print type(weights)
    #print weights.shape
    #print type(weights_1)
    """
    for i in range(tns_4):
    
        
        w = weights[2,:,:,i].copy()
        cv2.imwrite(path,w)
        NUM += 1
    #w = weights[:,:,::-1,0].copy()
    #plt.imshow(weights)
    
    """
