How to build:

`$ source <your_ros_installation_path>/setup.bash`

`$ catkin_make`

How to launch:

`$ roslaunch tortoisebot tortoisebot.launch model:=src/tortoisebot/urdf/tortoisebot.urdf`

Optionally, load provided configuration file (`configuration.rviz`) onto rviz.

======================================================================

# Tutorial ROS-Turtlebot-Gazebo-move_base

Fiz um tutorial bem curto de como como criar um mapa no gazebo com o turtlebot(SLAM) para depois utilizar o move_base com AMCL para deslocar no cenário.

Existem explicações no wiki ros que você também poderá consultar: 
http://wiki.ros.org/turtlebot_navigation/Tutorials/Build%20a%20map%20with%20SLAM
http://wiki.ros.org/turtlebot_navigation/Tutorials/Autonomously%20navigate%20in%20a%20known%20map

#### ALGUNS PACOTES A SEREM INSTALADOS - INCOMPLETO
sudo apt-get install ros-kinetic-desktop-full  
sudo apt-get install ros-kinetic-turtlebot_navigation  
sudo apt-get install ros-kinetic-turtlebot_gazebo  
sudo apt install ros-kinetic-turtlebot-rviz-launchers  

#### CRIANDO MAPA NO GAZEBO 
###### rodar cada comando num terminal:

1) roslaunch turtlebot_gazebo turtlebot_world.launch  
2) roslaunch turtlebot_gazebo gmapping_demo.launch  
3) roslaunch turtlebot_rviz_launchers view_navigation.launch  
4) roslaunch turtlebot_teleop keyboard_teleop.launch  

OBS Comando (1): roda o gazebo  
OBS Comando (2): armazena os dados num mapa na memória  
OBS Comando (3): roda o RVIZ para voce ver como estah ficando o mapa  
OBS Comando (4): controla a movimentaçao do robo via teclado. Movimente o robo pelo cenario. Quando o mapa no RVIZ parecer adequado/razoavel, voce deverá rodar o comando (5).  

5) rosrun map_server map_saver -f /home/marcelo/maps/my_new_map  

OBS Comando (5): Esse comando irá transferir o mapa da memória para o arquivo my_new_map.pgm. Além disso, será criado um arquivo de configuração my_new_map.yaml  
Se você quiser poderá corrigir o arquivo pgm no GIMP.


#### MOVENDO ROBO COM MAPA NO GAZEBO 
###### rodar cada comando num terminal:
1) roslaunch turtlebot_gazebo turtlebot_world.launch  
2) roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/marcelo/maps/my_new_map.yaml  
3) roslaunch turtlebot_rviz_launchers view_navigation.launch  

OBS Comando (1): roda o gazebo  
OBS Comando (2): carrega o mapa e o sistema de localização AMCL.  
OBS Comando (3): Roda o RVIZ, no qual você poderá definir comandos de movimentação por meio do botão "2D Nav Goal".
É provável que o turtlebot gire bastante. Ele faz isso por 2 motivos: estar tentando se localizar ou má configuração de deslocamento.
No segundo caso, você poderá reconfigurar o move_base do turtlebot (http://wiki.ros.org/base_local_planner).  

Também podemos fazer isso no robô real se você quiser.
