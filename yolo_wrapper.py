from ctypes import *
import math
import random

YOLOSO_PATH = "/home/noorvir/Documents/projects/libraries/darknet/libdarknet.so"


def sample(probs):
    s = sum(probs)
    probs = [a / s for a in probs]
    r = random.uniform(0, 1)
    for i in range(len(probs)):
        r = r - probs[i]
        if r <= 0:
            return i
    return len(probs) - 1


def c_array(ctype, values):
    arr = (ctype * len(values))()
    arr[:] = values
    return arr


class BOX(Structure):
    _fields_ = [("x", c_float),
                ("y", c_float),
                ("w", c_float),
                ("h", c_float)]


class DETECTION(Structure):
    _fields_ = [("bbox", BOX),
                ("classes", c_int),
                ("prob", POINTER(c_float)),
                ("mask", POINTER(c_float)),
                ("objectness", c_float),
                ("sort_class", c_int)]


class IMAGE(Structure):
    _fields_ = [("w", c_int),
                ("h", c_int),
                ("c", c_int),
                ("data", POINTER(c_float))]


class METADATA(Structure):
    _fields_ = [("classes", c_int),
                ("names", POINTER(c_char_p))]


class Network:
    """
    Helper class to ensure _out, _detections and _classifications stay in sync.
    """
    _net = None
    __out = None                # output of network
    _detections = None
    _classifications = None

    _load_net = None
    _load_meta = None
    _do_nms = None              # apply non-max suppression
    _free_image = None
    _free_detections = None
    _rgbgr = None               # reverse channels
    _meta = None                # info about the training data-set (classes etc)
    _predict_image = None       # method wrapper around network forward pass
    _make_network_boxes = None
    _get_network_boxes = None   # method wrapper around detector
    _letterbox_image = None

    @property
    def _out(self):
        return self.__out

    @_out.setter
    def _out(self, value):
        self.__out = value
        self._detections = None
        self._classifications = None


class YOLO:

    def __init__(self, config):

        self._config = config
        self._network = Network()
        self._lib = CDLL(YOLOSO_PATH, RTLD_GLOBAL)
        self._setup_network()

    def _setup_network(self):

        self._lib.network_width.argtypes = [c_void_p]
        self._lib.network_width.restype = c_int
        self._lib.network_height.argtypes = [c_void_p]
        self._lib.network_height.restype = c_int

        set_gpu = self._lib.cuda_set_device
        set_gpu.argtypes = [c_int]

        make_image = self._lib.make_image
        make_image.argtypes = [c_int, c_int, c_int]
        make_image.restype = IMAGE

        # Network
        self._network._load_net = self._lib.load_network
        self._network._load_net.argtypes = [c_char_p, c_char_p, c_int]
        self._network._load_net.restype = c_void_p

        # Meta
        self._load_meta = self._lib.get_metadata
        self._lib.get_metadata.argtypes = [c_char_p]
        self._lib.get_metadata.restype = METADATA

        # Image
        self._load_image = self._lib.load_image_color
        self._load_image.argtypes = [c_char_p, c_int, c_int]
        self._load_image.restype = IMAGE

        self._network._free_image = self._lib.free_image
        self._network._free_image.argtypes = [IMAGE]

        # Non-max Suppression
        self._network._do_nms = self._lib.do_nms_obj
        self._network._do_nms.argtypes = [POINTER(DETECTION),
                                          c_int, c_int, c_float]

        # Predictor
        self._network._predict_image = self._lib.network_predict_image
        self._network._predict_image.argtypes = [c_void_p, POINTER(c_float)]
        self._network._predict_image.restype = POINTER(c_float)

        self._network._rgbgr_image = self._lib.rgbgr_image
        self._network._rgbgr_image.argtypes = [IMAGE]

        # Detector
        self._network._get_network_boxes = self._lib.get_network_boxes
        self._network._get_network_boxes.argtypes = [c_void_p, c_int, c_int,
                                                     c_float, c_float,
                                                     POINTER(c_int), c_int,
                                                     POINTER(c_int)]
        self._network._get_network_boxes.restype = POINTER(DETECTION)

        self._network._free_detections = self._lib.free_detections
        self._network._free_detections.argtypes = [POINTER(DETECTION), c_int]

        self._network._make_network_boxes = self._lib.make_network_boxes
        self._network._make_network_boxes.argtypes = [c_void_p]
        self._network._make_network_boxes.restype = POINTER(DETECTION)

        self._network._free_ptrs = self._lib.free_ptrs
        self._network._free_ptrs.argtypes = [POINTER(c_void_p), c_int]

        self._network._letterbox_image = self._lib.letterbox_image
        self._network._letterbox_image.argtypes = [IMAGE, c_int, c_int]
        self._network._letterbox_image.restype = IMAGE

    def _predict(self, image):
        """Run network forward pass"""
        return self._network._predict_image(self._network._net, image)

    def _classify(self):
        """Run classification"""
        res = []
        for i in range(self._network._meta.classes):
            res.append((self._network._meta.names[i], self._network._out[i]))
        res = sorted(res, key=lambda x: -x[1])

        return res

    def _detect(self, w, h, thresh=.5, hier_thresh=.5, nms=.45):
        """Threshold predictions get detections"""
        num = c_int(0)
        pnum = pointer(num)
        dets = self._network._get_network_boxes(self._network._net,
                                                w, h, thresh, hier_thresh,
                                                None, 0, pnum)

        num = pnum[0]
        if nms:
            self._network._do_nms(dets, num, self._network._meta.classes, nms)

        res = []
        for j in range(num):
            for i in range(self._network._meta.classes):
                if dets[j].prob[i] > 0:
                    b = dets[j].bbox
                    res.append((self._network._meta.names[i], dets[j].prob[i],
                                (b.x, b.y, b.w, b.h)))

        res = sorted(res, key=lambda x: -x[1])
        self._network._free_detections(dets, num)

        return res

    def start(self):
        self._network._net = self._network._load_net(self._config['cfg'],
                                                     self._config['weights'], 0)
        self._network._meta = self._network._load_meta()

    def run(self, image, thresh=.5, hier_thresh=.5, nms=.45):
        """

        Parameters
        ----------
        image
        thresh
        hier_thresh
        nms

        Returns
        -------

        """
        # TODO: implement passing preloaded image
        # TODO: perform checks on image type etc
        # TODO: perform conversions if necessary
        if type(image) == str:
            image = self._load_image(image, 0, 0)
        else:
            pass

        # Predict (Forward Pass)
        self._network._out = self._predict(image)

        # Classify
        self._network._classifications = self._classify()

        # Detect
        self._network._detections = self._detect(image.w, image.h, thresh,
                                                 hier_thresh, nms)

        self._network._free_image(image)

    def get_predictions(self):
        """Returns raw network output"""
        return self._network._out

    def get_classifications(self):
        """Returns classification scores"""
        return self._network._classifications

    def get_detections(self):
        """Returns detections boxes"""
        return self._network._detections


if __name__ == "__main__":
    yolo = YOLO()
    yolo.start()
    yolo.run("data/dog.jpg")
    # net = load_net("cfg/tiny-yolo.cfg", "tiny-yolo.weights", 0)
    # meta = load_meta("cfg/coco.data")
    print(yolo.get_detections())
