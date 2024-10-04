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
                # TODO:declare MPCC params

                # MPCC params
                ('controllers.MPCC.N', rclpy.Parameter.Type.INTEGER),
                ('controllers.MPCC.Tf', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.q_con', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.q_long', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.q_theta', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.q_d', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.q_delta', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.delta_max', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.d_max', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.d_min', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.ddot_max', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.deltadot_max', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.thetahatdot_min', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.thetahatdot_max', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.opt_tol', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.max_QP_iter', rclpy.Parameter.Type.INTEGER),
                ('controllers.MPCC.freq_limit', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.res_limit', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.mu_xi', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC.mu_eta', rclpy.Parameter.Type.DOUBLE),

                #reverse MPCC params:
                ('controllers.MPCC_reverse.N', rclpy.Parameter.Type.INTEGER),
                ('controllers.MPCC_reverse.Tf', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.q_con', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.q_long', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.q_theta', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.q_d', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.q_delta', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.delta_max', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.d_max', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.d_min', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.ddot_max', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.deltadot_max', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.thetahatdot_min', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.thetahatdot_max', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.opt_tol', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.max_QP_iter', rclpy.Parameter.Type.INTEGER),
                ('controllers.MPCC_reverse.freq_limit', rclpy.Parameter.Type.DOUBLE),
                ('controllers.MPCC_reverse.res_limit', rclpy.Parameter.Type.DOUBLE),

                
                # vehicle params
                ('vehicle_params.C_m1' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.C_m2' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.C_m3' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.m',rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.C_f' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.C_r' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.l_f' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.l_r' ,rclpy.Parameter.Type.DOUBLE),
                ('vehicle_params.I_z' ,rclpy.Parameter.Type.DOUBLE)
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
    MPCC_params = {
        "N": loader.get_parameter("controllers.MPCC.N").value,
        "Tf": loader.get_parameter("controllers.MPCC.Tf").value,
        "q_con": loader.get_parameter("controllers.MPCC.q_con").value,
        "q_long": loader.get_parameter("controllers.MPCC.q_long").value,
        "q_theta": loader.get_parameter("controllers.MPCC.q_theta").value,
        "q_d": loader.get_parameter("controllers.MPCC.q_d").value,
        "q_delta": loader.get_parameter("controllers.MPCC.q_delta").value,
        "delta_max": loader.get_parameter("controllers.MPCC.delta_max").value,
        "d_max": loader.get_parameter("controllers.MPCC.d_max").value,
        "d_min": loader.get_parameter("controllers.MPCC.d_min").value,
        "ddot_max": loader.get_parameter("controllers.MPCC.ddot_max").value,
        "deltadot_max": loader.get_parameter("controllers.MPCC.deltadot_max").value,
        "thetahatdot_min": loader.get_parameter("controllers.MPCC.thetahatdot_min").value,
        "thetahatdot_max": loader.get_parameter("controllers.MPCC.thetahatdot_max").value,
        "opt_tol": loader.get_parameter("controllers.MPCC.opt_tol").value,
        "max_QP_iter": loader.get_parameter("controllers.MPCC.max_QP_iter").value,
        "freq_limit": loader.get_parameter("controllers.MPCC.freq_limit").value,
        "res_limit": loader.get_parameter("controllers.MPCC.res_limit").value,
        "mu_xi": loader.get_parameter("controllers.MPCC.mu_xi").value,
        "mu_eta": loader.get_parameter("controllers.MPCC.mu_eta").value,

    }
    # TODO: collect params into dict and pass to tha manager as kwargs

    MPCC_reverse_params= {
        "N": loader.get_parameter("controllers.MPCC_reverse.N").value,
        "Tf": loader.get_parameter("controllers.MPCC_reverse.Tf").value,
        "q_con": loader.get_parameter("controllers.MPCC_reverse.q_con").value,
        "q_long": loader.get_parameter("controllers.MPCC_reverse.q_long").value,
        "q_theta": loader.get_parameter("controllers.MPCC_reverse.q_theta").value,
        "q_d": loader.get_parameter("controllers.MPCC_reverse.q_d").value,
        "q_delta": loader.get_parameter("controllers.MPCC_reverse.q_delta").value,
        "delta_max": loader.get_parameter("controllers.MPCC_reverse.delta_max").value,
        "d_max": loader.get_parameter("controllers.MPCC_reverse.d_max").value,
        "d_min": loader.get_parameter("controllers.MPCC_reverse.d_min").value,
        "ddot_max": loader.get_parameter("controllers.MPCC_reverse.ddot_max").value,
        "deltadot_max": loader.get_parameter("controllers.MPCC_reverse.deltadot_max").value,
        "thetahatdot_min": loader.get_parameter("controllers.MPCC_reverse.thetahatdot_min").value,
        "thetahatdot_max": loader.get_parameter("controllers.MPCC_reverse.thetahatdot_max").value,
        "opt_tol": loader.get_parameter("controllers.MPCC_reverse.opt_tol").value,
        "max_QP_iter": loader.get_parameter("controllers.MPCC_reverse.max_QP_iter").value,
        "freq_limit": loader.get_parameter("controllers.MPCC_reverse.freq_limit").value,
        "res_limit": loader.get_parameter("controllers.MPCC_reverse.res_limit").value
    }


    vehicle_params = {
        'm': loader.get_parameter("vehicle_params.m").value,
        'C_f': loader.get_parameter("vehicle_params.C_f").value,
        'C_r': loader.get_parameter("vehicle_params.C_r").value,
        'l_f': loader.get_parameter("vehicle_params.l_f").value,
        'l_r': loader.get_parameter("vehicle_params.l_r").value,
        'C_m1': loader.get_parameter("vehicle_params.C_m1").value,
        'C_m2': loader.get_parameter("vehicle_params.C_m2").value,
        'C_m3': loader.get_parameter("vehicle_params.C_m3").value,
        'I_z': loader.get_parameter("vehicle_params.I_z").value,

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
                           GP_LPV_LQR_params=GP_LPV_LQR_params,
                           MPCC_params = MPCC_params,
                           MPCC_reverse_params = MPCC_reverse_params)
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