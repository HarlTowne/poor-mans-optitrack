import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from marker_msgs.msg import MatchedMarkers
from struct import pack
from sensor_msgs_py import point_cloud2

class _3Dify(Node):
    def __init__(self):
        super().__init__('_3Difier')
        self._2d_points_sub = self.create_subscription(MatchedMarkers, "/matched_markers", self._recv_points, 1)
        self._out_pub = self.create_publisher(PointCloud2, "cam_ref/markers", 1)

    def _recv_points(self, msg: MatchedMarkers):
        if len(msg.left) == 0 or len(msg.right) == 0 or len(msg.left) != len(msg.right):
            print("length error?")
            return
        print(len(msg.left))
        x_field = PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1)
        y_field = PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1)
        z_field = PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1)
        data_float = []
        for i in range(len(msg.left)):
            # u = camera x
            # v = camera y (down positive)
            # params hard coppied
            ul = (msg.left[i].x-659.31299)/894.47602
            vl = (msg.left[i].y-435.75521)/895.95297
            ur = (msg.right[i].x-654.7363)/907.21913
            vr = (msg.right[i].y-393.90347)/908.33261
            ud = ul-ur

            z = 125/ud /1000
            y = (vl*z + vr*z)/2# avg
            x = ((ul*z + 125/2) + (ur*z - 125/2))/2 #avg
            data_float.extend([x, y, z])
        try:
            data_bytearray = pack("<{}f".format(len(msg.right)*3), *data_float)
        except Exception as e:
            print(e, "<{}f".format(len(msg.right)*3))
        new_header: Header=msg.header
        new_header.frame_id = "map"
        out_msg = PointCloud2(header=new_header, is_dense=True, point_step=4*3, fields=[x_field, y_field, z_field], is_bigendian=False,
                              width=len(msg.left), height=1, row_step=4*3*len(msg.left), data=data_bytearray)
        pc2_data = point_cloud2.read_points_list(out_msg, skip_nans=True)
        # print(data_float)
        # print(pc2_data)
        self._out_pub.publish(out_msg)
                              
        

def main(args = None):
    rclpy.init(args=args)
    marker_detector = _3Dify()
    try:
        rclpy.spin(marker_detector)
    except:
        pass
    marker_detector.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
