import tensorflow as tf
import numpy as np
import cv2

from styx_msgs.msg import TrafficLight


def load_graph(frozen_graph_filename):
    # We load the protobuf file from the disk and parse it to retrieve the 
    # unserialized graph_def
    with tf.gfile.GFile(frozen_graph_filename, "rb") as f:
        graph_def = tf.GraphDef()
        graph_def.ParseFromString(f.read())

    # Then, we import the graph_def into a new Graph and returns it 
    with tf.Graph().as_default() as graph:
        # The name var will prefix every op/nodes in your graph
        # Since we load everything in a new graph, this is not needed
        tf.import_graph_def(graph_def, name="prefix")
    
    return graph


def rgb2gray(x):
    return np.dot(x[...,:3], [0.299, 0.583, 0.114])

### Preprocess the data here. Preprocessing steps could include normalization, converting to grayscale, etc.
### Feel free to use as many code cells as needed
def normalize(x):
    """
    Normalize a list of sample image data in the range of 0 to 1
    : x: List of image data.  The image shape is (32, 32, 3)
    : return: Numpy array of normalize data
    """
    
    x_norm = []

    for img in x:
        x_norm.append(img/255)
    
    return x_norm

def grayscale(x):
    """Applies the Grayscale transform
    This will return an image with only one color channel
    but NOTE: to see the returned image as grayscale
    (assuming your grayscaled image is called 'gray')
    you should call plt.imshow(gray, cmap='gray')"""
    x_gray = []
    for img in x:
        img2 = cv2.cvtColor(img, cv2.COLOR_RGB2GRAY)
        x_gray.append(img2)
        
    return np.array(x_gray)
    # Or use BGR2GRAY if you read an image with cv2.imread()
    #return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def reshape(x):
    '''
    preprocess

    '''
    img2 = []
    for img in x:
        img_reshaped = img.reshape((32, 32, 1))
        img2.append(img_reshaped)
    return np.array(img2)

def one_hot_encode(x, n_classes):
    """
    preprocess
    One hot encode a list of sample labels. Return a one-hot encoded vector for each label.
    : x: List of sample Labels
    : n_classes: Number of classes
    : return: Numpy array of one-hot encoded labels
    """
    # TODO: Implement Function
    all = []
    
    for label in x:
#        print(label)
        
        # create vector with zeros
        lbl_vec = np.zeros(n_classes)
        
        # set the appropriate value to 1.
        lbl_vec[label] = 1.
        
#        print(lbl_vec)
        
        all.append(lbl_vec)
    
#    print(all)
    return np.array(all)

def preprocess(x):
    '''
    : x: features vector
    : y: labels vector
    '''
    x = rgb2gray(x)

    x = normalize(x)
    
    x = reshape(x)

    
    return(x.astype(np.float32))


def normalize_logits(logits):
    """
    Normalize a list of sample image data in the range of 0 to 1
    : x: List of image data.  The image shape is (32, 32, 3)
    : return: Numpy array of normalize data
    """
    
    logits_norm = np.array(logits, copy=True) 


    # set negative values to 0
    for i in range(0, len(logits)):

        for j in range(0, len(logits[i])):
            logits_norm[i][j] = max(0, logits[i][j])

        #minv = np.amin(logits[i])
        #print("minv="+str(minv))

        #logits_norm[i] = (logits[i] - minv)

        #rint("shifted logits " + str(logits_norm))
    
    for i in range(0, len(logits_norm)):

        #maxv = np.amax(logits_norm[i])
        #print("max="+str(maxv))

        sum = np.sum(logits_norm[i])
        #print("sum="+str(sum))

        logits_norm[i] = (logits_norm[i] / sum)

        #print("normalized logits " + str(logits_norm))

    return logits_norm


class TLClassifier(object):
    def __init__(self, model_file):
        #TODO load classifier
        
        # load frozen graph
        self.graph = load_graph(model_file)

        # access the input and output nodes 
        self.loaded_x = self.graph.get_tensor_by_name('prefix/x:0')
        self.loaded_y = self.graph.get_tensor_by_name('prefix/y:0')
        self.loaded_keep_prob = self.graph.get_tensor_by_name('prefix/keep_prob:0')
        self.loaded_logits = self.graph.get_tensor_by_name('prefix/logits:0')
        self.loaded_acc = self.graph.get_tensor_by_name('prefix/accuracy:0')



    def get_classification(self, img_cutout):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        # resize image to 32x32
	#print img_cutout.shape

        image = cv2.resize(img_cutout, (32, 32))

        # create a batch
        batch = []
        batch.append(image)
        batch = np.asarray(batch)

        # preprocess batch
        batch = preprocess(batch)


        # launch a Session
        with tf.Session(graph=self.graph) as sess:
            # Note: we don't nee to initialize/restore anything
            # There is no Variables in this graph, only hardcoded constants 
            logits = sess.run(self.loaded_logits, feed_dict={self.loaded_x: batch, self.loaded_y: np.ndarray(shape=(1,3)), self.loaded_keep_prob: 1.0})
            # I taught a neural net to recognise when a sum of numbers is bigger than 45
            # it should return False in this case

            # normalize logits
            logits = normalize_logits(logits)

            # get id with highest value
            if ( (logits[0][0] > logits[0][1]) and (logits[0][0] > logits[0][2]) ):
                return TrafficLight.GREEN
            elif ( (logits[0][1] > logits[0][0]) and (logits[0][1] > logits[0][2]) ):
                return TrafficLight.YELLOW
            elif ( (logits[0][2] > logits[0][0]) and (logits[0][2] > logits[0][1]) ):
                return TrafficLight.RED

            return TrafficLight.UNKNOWN
