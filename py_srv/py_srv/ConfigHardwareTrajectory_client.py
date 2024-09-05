import rclpy
from rclpy.node import Node
from exo_interfaces.srv import ConfigHardwareTrajectory
import yaml

# EXPLICACIÓN:
# Este codigo se encarga de hacer un cliente, recoger los valores de las variables de hardware y de la trayectoria.yaml y crear un vector para hip y para knee
# Se han creado dos servidos conectados a este cliente. Uno en python encargado de establecer conexión con el canbus, y otro en c++ encargado de la configuracion de SPIbus.


class ConfigHardwareTrajectoryClient(Node):

    def __init__(self):
        super().__init__('configuration_client')
        self.cli = self.create_client(ConfigHardwareTrajectory, 'configuration')
        #while not self.cli.wait_for_service(timeout_sec=1.0):
        #    self.get_logger().info('Service not available, waiting again...')
        self.req = ConfigHardwareTrajectory.Request()

    def send_request(self, verbose, elevation_port, exo_port, sampling_frequency, right_knee_can_id, right_knee_potentiometer, right_knee_gauge, right_knee_fsr1, right_knee_fsr2, left_knee_can_id, left_knee_potentiometer, left_knee_gauge, left_knee_fsr1, left_knee_fsr2, right_hip_can_id, right_hip_potentiometer, right_hip_gauge, right_hip_fsr1, right_hip_fsr2, left_hip_can_id, left_hip_potentiometer, left_hip_gauge, left_hip_fsr1, left_hip_fsr2, size, hip_trajectory_vector, knee_trajectory_vector):
        # DEfinimos las variables como tenemos en el archivo.srv
        try:
            ############################ VARIABLES OF HARDWARE#########################3
            # Para las variables booleanas estaba teniendo problemas si ponia un int en el yaml, por lo que he decidido poner una funcion que verifique si tiene un balor booleano, en caso contrario que devuelva False
            # Ademas, se ha implementado que si el can_id no es el correspondiente en el joint, los sensores permaneceran en false, en caso contrario se cogeran los valores booleanos correctos
            self.req.verbose = self.convert_to_bool(verbose)
            self.req.elevation_port = str(elevation_port)
            self.req.exo_port = str(exo_port)
            self.req.sampling_frequency = int(sampling_frequency)
            # right_knee
            self.req.right_knee_can_id = int(right_knee_can_id)
            if  self.req.right_knee_can_id == 95:
                self.req.right_knee_potentiometer = self.convert_to_bool(right_knee_potentiometer)
                self.req.right_knee_gauge = self.convert_to_bool(right_knee_gauge)
                self.req.right_knee_fsr1 = self.convert_to_bool(right_knee_fsr1)
                self.req.right_knee_fsr2 = self.convert_to_bool(right_knee_fsr2)
            else:
                self.req.right_knee_potentiometer = bool(False)
                self.req.right_knee_gauge = bool(False)
                self.req.right_knee_fsr1 = bool(False)
                self.req.right_knee_fsr2 = bool(False)
            # left_knee
            self.req.left_knee_can_id = int(left_knee_can_id)
            if  self.req.left_knee_can_id == 90:
                self.req.left_knee_potentiometer = self.convert_to_bool(left_knee_potentiometer)
                self.req.left_knee_gauge = self.convert_to_bool(left_knee_gauge)
                self.req.left_knee_fsr1 = self.convert_to_bool(left_knee_fsr1)
                self.req.left_knee_fsr2 = self.convert_to_bool(left_knee_fsr2)
            else:
                self.req.left_knee_potentiometer = bool(False)
                self.req.left_knee_gauge = bool(False)
                self.req.left_knee_fsr1 = bool(False)
                self.req.left_knee_fsr2 = bool(False)
            # right_hip
            self.req.right_hip_can_id = int(right_hip_can_id)
            if  self.req.right_hip_can_id == 85:
                self.req.right_hip_potentiometer = self.convert_to_bool(right_hip_potentiometer)
                self.req.right_hip_gauge = self.convert_to_bool(right_hip_gauge)
                self.req.right_hip_fsr1 = self.convert_to_bool(right_hip_fsr1)
                self.req.right_hip_fsr2 = self.convert_to_bool(right_hip_fsr2)
            else:
                self.req.right_hip_potentiometer = bool(False)
                self.req.right_hip_gauge = bool(False)
                self.req.right_hip_fsr1 = bool(False)
                self.req.right_hip_fsr2 = bool(False)
            # left_hip
            self.req.left_hip_can_id = int(left_hip_can_id)
            if  self.req.left_hip_can_id == 80:
                self.req.left_hip_potentiometer = self.convert_to_bool(left_hip_potentiometer)
                self.req.left_hip_gauge = self.convert_to_bool(left_hip_gauge)
                self.req.left_hip_fsr1 = self.convert_to_bool(left_hip_fsr1)
                self.req.left_hip_fsr2 = self.convert_to_bool(left_hip_fsr2)
            else:
                self.req.left_hip_potentiometer = bool(False)
                self.req.left_hip_gauge = bool(False)
                self.req.left_hip_fsr1 = bool(False)
                self.req.left_hip_fsr2 = bool(False)

            ################### VARIABLES OF TRAJECTORY ############################
            self.req.size = int(size)
            self.req.hip_trajectory_vector = hip_trajectory_vector
            self.req.knee_trajectory_vector = knee_trajectory_vector

            future = self.cli.call_async(self.req)
            rclpy.spin_until_future_complete(self, future)

            result = future.result()
            if result is not None:
                success = result.success
                self.get_logger().info(f"Success: {success}")
            else:
                self.get_logger().warn('Service call failed')

        except Exception as e:
            self.get_logger().error(f"Error: {e}")
            success = False
            self.get_logger().error(f"Success: {success}")

    ################### READ VARIABLES OF HARDWARE ###################
    def read_hardware_configuration_params(self, file_path):
        with open(file_path, 'r') as file:
            try:
                hardware_configuration_params = yaml.safe_load(file)
                verbose = hardware_configuration_params['verbose']
                elevation_port = hardware_configuration_params['can_comm']['elevation_port']
                exo_port = hardware_configuration_params['can_comm']['exo_port']
                sampling_frequency = hardware_configuration_params['exo_hw']['sampling_frequency']
                right_knee_can_id = hardware_configuration_params['exo_hw']['joints']['right_knee']['can_id']
                right_knee_potentiometer = hardware_configuration_params['exo_hw']['joints']['right_knee']['Potentiometer']
                right_knee_gauge = hardware_configuration_params['exo_hw']['joints']['right_knee']['Gauge']
                right_knee_fsr1 = hardware_configuration_params['exo_hw']['joints']['right_knee']['FSR1']
                right_knee_fsr2 = hardware_configuration_params['exo_hw']['joints']['right_knee']['FSR2']
                left_knee_can_id = hardware_configuration_params['exo_hw']['joints']['left_knee']['can_id']
                left_knee_potentiometer = hardware_configuration_params['exo_hw']['joints']['left_knee']['Potentiometer']
                left_knee_gauge = hardware_configuration_params['exo_hw']['joints']['left_knee']['Gauge']
                left_knee_fsr1 = hardware_configuration_params['exo_hw']['joints']['left_knee']['FSR1']
                left_knee_fsr2 = hardware_configuration_params['exo_hw']['joints']['left_knee']['FSR2']
                right_hip_can_id = hardware_configuration_params['exo_hw']['joints']['right_hip']['can_id']
                right_hip_potentiometer = hardware_configuration_params['exo_hw']['joints']['right_hip']['Potentiometer']
                right_hip_gauge = hardware_configuration_params['exo_hw']['joints']['right_hip']['Gauge']
                right_hip_fsr1 = hardware_configuration_params['exo_hw']['joints']['right_hip']['FSR1']
                right_hip_fsr2 = hardware_configuration_params['exo_hw']['joints']['right_hip']['FSR2']
                left_hip_can_id = hardware_configuration_params['exo_hw']['joints']['left_hip']['can_id']
                left_hip_potentiometer = hardware_configuration_params['exo_hw']['joints']['left_hip']['Potentiometer']
                left_hip_gauge = hardware_configuration_params['exo_hw']['joints']['left_hip']['Gauge']
                left_hip_fsr1 = hardware_configuration_params['exo_hw']['joints']['left_hip']['FSR1']
                left_hip_fsr2 = hardware_configuration_params['exo_hw']['joints']['left_hip']['FSR2']
                return verbose, elevation_port, exo_port, sampling_frequency, right_knee_can_id, right_knee_potentiometer, right_knee_gauge, right_knee_fsr1, right_knee_fsr2, left_knee_can_id, left_knee_potentiometer, left_knee_gauge, left_knee_fsr1, left_knee_fsr2, right_hip_can_id, right_hip_potentiometer, right_hip_gauge, right_hip_fsr1, right_hip_fsr2, left_hip_can_id, left_hip_potentiometer, left_hip_gauge, left_hip_fsr1, left_hip_fsr2
            except yaml.YAMLError as exc:
                print(f"Error al cargar el archivo YAML: {exc}")
                return None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None, None

    def convert_to_bool(self, value):
        if isinstance(value, bool):
            return value
        elif isinstance(value, str):
            return value.lower() in ['true', 'yes', '1']
        else:
            return False  # o manejar el caso según tus necesidades

    ################### READ VARIABLES OF TRAJECTORY ################33
    def read_trajectory_params(self, file_path):
        try:
            with open(file_path, 'r') as file:
                trajectory_params = yaml.safe_load(file)
                size = trajectory_params['trajectory']['size']
                #print(f"DEBUG: size={size}, type(size)={type(size)}")
                #self.get_logger().info("Size: {}".format(size))
                if size is not None:
                    hip_trajectory_vector = []
                    knee_trajectory_vector = []

                    # Creamos la lista con los vectores
                    for i in range(size):
                        i_key = str(i)
                        if i_key in trajectory_params['trajectory']['hip_knee_positions']:
                            hip_key = trajectory_params['trajectory']['hip_knee_positions'][i_key]["hip"]
                            knee_key = trajectory_params['trajectory']['hip_knee_positions'][i_key]["knee"]
                            
                            # Se comprueba que hip_key y knee_key tienen un valor distinto de None
                            if hip_key is not None and knee_key is not None:
                                hip_trajectory_vector.append(hip_key)
                                knee_trajectory_vector.append(knee_key)

                            else:
                                # Si es None puede ser que no tenga valor asignado o que hip/knee no esten definidos en ese i_key en el archivo.yaml
                                self.get_logger().error(f"Error in parameter: '{i_key}'.")
                                if hip_key is None:
                                    self.get_logger().warn(f"Hip parameter not found in i_key {i_key}.")
                                if knee_key is None:
                                    self.get_logger().warn(f"Knee parameter not found in i_key {i_key}.")
                        else:
                            # Estos comentarios se muestran por pantalla cuando algún i_key no se encuentra definido en el archivo.yaml
                            self.get_logger().error(f"Parameter in i_key '{i_key}' is not found .")

                    return size, hip_trajectory_vector, knee_trajectory_vector
                else:
                    self.get_logger().error("Error Size is None ")

        except yaml.YAMLError as exc:
            self.get_logger().error(f"Error loading YAML file: {exc}")
            self.get_logger().error(f"Error while loading YAML file at path: {file_path}")
            return None, None, None

def main(args=None):
    rclpy.init(args=args)
    configuration_client = ConfigHardwareTrajectoryClient()

    # FILE YAML PATH:
    hardware_configuration_yaml_path = "/home/ubuntu/ros2_exo_ws/src/py_srv/config/hardware_configuration.yaml"
    verbose, elevation_port, exo_port, sampling_frequency, right_knee_can_id, right_knee_potentiometer, right_knee_gauge, right_knee_fsr1, right_knee_fsr2, left_knee_can_id, left_knee_potentiometer, left_knee_gauge, left_knee_fsr1, left_knee_fsr2, right_hip_can_id, right_hip_potentiometer, right_hip_gauge, right_hip_fsr1, right_hip_fsr2, left_hip_can_id, left_hip_potentiometer, left_hip_gauge, left_hip_fsr1, left_hip_fsr2 = configuration_client.read_hardware_configuration_params(hardware_configuration_yaml_path)
    trajectory_yaml_path = "/home/ubuntu/ros2_exo_ws/src/py_srv/config/trajectory.yaml"
    size, hip_trajectory_vector, knee_trajectory_vector = configuration_client.read_trajectory_params(trajectory_yaml_path)

     # Si todas las variables tienen un valor distinto de None, se envían los datos
    if None not in (verbose, elevation_port, exo_port, sampling_frequency, right_knee_can_id, right_knee_potentiometer, right_knee_gauge, right_knee_fsr1, right_knee_fsr2, left_knee_can_id, left_knee_potentiometer, left_knee_gauge, left_knee_fsr1, left_knee_fsr2, right_hip_can_id, right_hip_potentiometer, right_hip_gauge, right_hip_fsr1, right_hip_fsr2, left_hip_can_id, left_hip_potentiometer, left_hip_gauge, left_hip_fsr1, left_hip_fsr2) and len(hip_trajectory_vector) == len(knee_trajectory_vector) == size and all(hip is not None and knee is not None for hip, knee in zip(hip_trajectory_vector, knee_trajectory_vector)):
         configuration_client.send_request(verbose, elevation_port, exo_port, sampling_frequency, right_knee_can_id, right_knee_potentiometer, right_knee_gauge, right_knee_fsr1, right_knee_fsr2, left_knee_can_id, left_knee_potentiometer, left_knee_gauge, left_knee_fsr1, left_knee_fsr2, right_hip_can_id, right_hip_potentiometer, right_hip_gauge, right_hip_fsr1, right_hip_fsr2, left_hip_can_id, left_hip_potentiometer, left_hip_gauge, left_hip_fsr1, left_hip_fsr2, size, hip_trajectory_vector, knee_trajectory_vector)
    # En el caso de que alguna variable le falte el valor, succes es false y se imprimen los siguientes mensajes.
    else:
        configuration_client.get_logger().warn('Request not sent due to missing parameters or due to incorrect vector lengths or missing values')
        success = False
        configuration_client.get_logger().error(f"Success: {success}")

    

    rclpy.spin(configuration_client)
    configuration_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

