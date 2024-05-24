import rclpy
from rclpy.node import Node
from .manager import ControlManager
import traceback

class LoaderNode(Node):
    def __init__(self):
        super().__init__("parameter_server")

        
        self.declare_parameters(
            namespace= "",
            parameters=[
                ('car_id', rclpy.Parameter.Type.STRING),
                # lpv lq 
                ("controllers.LPV_LQR.FREQUENCY", rclpy.Parameter.Type.DOUBLE),
                ('controllers.LPV_LQR.LATERAL_CONTROL_GAINS.k1',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.LPV_LQR.LATERAL_CONTROL_GAINS.k2',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.LPV_LQR.LATERAL_CONTROL_GAINS.k3',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.LPV_LQR.LATERAL_CONTROL_GAINS.k1_r',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.LPV_LQR.LATERAL_CONTROL_GAINS.k2_r',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.LPV_LQR.LONGITUDINAL_CONTROL_GAINS.k1' ,rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.LPV_LQR.LONGITUDINAL_CONTROL_GAINS.k2' ,rclpy.Parameter.Type.DOUBLE_ARRAY),
                
                # gp lpv lq 
                ("controllers.GP_LPV_LQR.GP_TYPE", rclpy.Parameter.Type.STRING),
                ("controllers.GP_LPV_LQR.FREQUENCY", rclpy.Parameter.Type.DOUBLE),
                ('controllers.GP_LPV_LQR.LATERAL_CONTROL_GAINS.k1',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.GP_LPV_LQR.LATERAL_CONTROL_GAINS.k2',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.GP_LPV_LQR.LATERAL_CONTROL_GAINS.k3',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.GP_LPV_LQR.LATERAL_CONTROL_GAINS.k1_r',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.GP_LPV_LQR.LATERAL_CONTROL_GAINS.k2_r',rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.GP_LPV_LQR.LONGITUDINAL_CONTROL_GAINS.k1' ,rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.GP_LPV_LQR.LONGITUDINAL_CONTROL_GAINS.k2' ,rclpy.Parameter.Type.DOUBLE_ARRAY),
                ('controllers.GP_LPV_LQR.NUM_OF_INDUCING' ,rclpy.Parameter.Type.INTEGER),
                ('controllers.GP_LPV_LQR.BATCH_SIZE' ,rclpy.Parameter.Type.INTEGER),
                ('controllers.GP_LPV_LQR.RETRAIN_ITER' ,rclpy.Parameter.Type.INTEGER),
                ('controllers.GP_LPV_LQR.FORGETTING_FACTOR' ,rclpy.Parameter.Type.DOUBLE),
                ('controllers.GP_LPV_LQR.CONFIDENCE' ,rclpy.Parameter.Type.DOUBLE),

                #TODO
                # include the parameters of any other controller implemented
                

                # vehicle params
                ('vehicle_params.C_m1' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.C_m2' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.C_m3' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.m',rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.C_f' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.C_r' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.l_f' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.l_r' ,rclpy.Parameter.Type.DOUBLE)
            ]
        )
        
def main():
    rclpy.init()
    loader = LoaderNode()

    car_ID = loader.get_parameter('car_id').value
    
    LPV_LQR_params = {
        "frequency": loader.get_parameter("controllers.LPV_LQR.FREQUENCY").value,
        "lat_gains" : {
            'k1': loader.get_parameter("controllers.LPV_LQR.LATERAL_CONTROL_GAINS.k1").value,
            'k2': loader.get_parameter("controllers.LPV_LQR.LATERAL_CONTROL_GAINS.k2").value,
            'k3': loader.get_parameter("controllers.LPV_LQR.LATERAL_CONTROL_GAINS.k3").value,
            'k1_r': loader.get_parameter("controllers.LPV_LQR.LATERAL_CONTROL_GAINS.k1_r").value,
            'k2_r': loader.get_parameter("controllers.LPV_LQR.LATERAL_CONTROL_GAINS.k2_r").value
            },
        "long_gains" :{
            'k1': loader.get_parameter("controllers.LPV_LQR.LONGITUDINAL_CONTROL_GAINS.k1").value,
            'k2': loader.get_parameter("controllers.LPV_LQR.LONGITUDINAL_CONTROL_GAINS.k2").value
          }
        }

    GP_LPV_LQR_params = {
        "GP_type": loader.get_parameter("controllers.GP_LPV_LQR.GP_TYPE").value,
        "frequency": loader.get_parameter("controllers.GP_LPV_LQR.FREQUENCY").value,
        "num_of_inducing": loader.get_parameter("controllers.GP_LPV_LQR.NUM_OF_INDUCING").value,
        "forgetting_factor": loader.get_parameter("controllers.GP_LPV_LQR.FORGETTING_FACTOR").value,
        "confidence_level": loader.get_parameter("controllers.GP_LPV_LQR.CONFIDENCE").value,
        "batch_size": loader.get_parameter("controllers.GP_LPV_LQR.BATCH_SIZE").value,
        "retrain_iter": loader.get_parameter("controllers.GP_LPV_LQR.RETRAIN_ITER").value,
        "lat_gains" : {
            'k1': loader.get_parameter("controllers.GP_LPV_LQR.LATERAL_CONTROL_GAINS.k1").value,
            'k2': loader.get_parameter("controllers.GP_LPV_LQR.LATERAL_CONTROL_GAINS.k2").value,
            'k3': loader.get_parameter("controllers.GP_LPV_LQR.LATERAL_CONTROL_GAINS.k3").value,
            'k1_r': loader.get_parameter("controllers.GP_LPV_LQR.LATERAL_CONTROL_GAINS.k1_r").value,
            'k2_r': loader.get_parameter("controllers.GP_LPV_LQR.LATERAL_CONTROL_GAINS.k2_r").value
            },
        "long_gains" :{
            'k1': loader.get_parameter("controllers.GP_LPV_LQR.LONGITUDINAL_CONTROL_GAINS.k1").value,
            'k2': loader.get_parameter("controllers.GP_LPV_LQR.LONGITUDINAL_CONTROL_GAINS.k2").value

          }
        }
    
    # TODO: collect params into dict and pass to tha manager as kwargs



    vehicle_params = {
        'm': loader.get_parameter("vehicle_params.m").value,
        'C_f': loader.get_parameter("vehicle_params.C_f").value,
        'C_r': loader.get_parameter("vehicle_params.C_r").value,
        'l_f': loader.get_parameter("vehicle_params.l_f").value,
        'l_r': loader.get_parameter("vehicle_params.l_r").value,
        'C_m1': loader.get_parameter("vehicle_params.C_m1").value,
        'C_m2': loader.get_parameter("vehicle_params.C_m2").value,
        'C_m3': loader.get_parameter("vehicle_params.C_m3").value,

    }

    # TODO: this should be parameter
    TCP_params = {
        'host': '192.168.2.62',
        'port': 8069
    }
    
    loader.destroy_node()
    manager=ControlManager(car_ID = car_ID,
                           TCP_params = TCP_params,
                           vehicle_params = vehicle_params,
                           LPV_LQR_params=LPV_LQR_params,
                           GP_LPV_LQR_params=GP_LPV_LQR_params)
    try:
        rclpy.spin(manager)
    except Exception:
        manager.get_logger().error(traceback.format_exc())
    finally:
        manager.TCP_thread.join()
        manager.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()