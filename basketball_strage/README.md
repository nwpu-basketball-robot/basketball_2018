所有状态机文件均在scripts文件夹下

目录列表：
scripts:
    -robot_find_pkg(摄像头的相关接口的目录)
                find_line.py （检测边线的接口）
        find_volleyball.py （检测排球的接口）

    -robot_move_pkg（机器人移动的相关接口的目录）
        -interpolation_function （关于速度插值的算法的目录）
            cubic_spline.py
            growth_curve.py
            spline_config.py
        config.py （机器人移动时的各种速度及阙值参数文件）
        go_along_circle.py （机器人进行圆弧运动的接口）
        go_close_line.py （机器人接近边线并回到初始点的接口）
        linear_move.py （机器人以世界坐标系移动的接口）
        low_speed_linear_move.py （机器人以世界坐标系移动的接口，较慢速度）
        move_a_distance.py 
        move_in_robot.py （机器人以机器人坐标系移动的接口））
        turn_an_angular.py （机器人转一定角度的接口）

    -robot_shovel_srv（铲子控制的接口的目录）
        control_srv.py （铲子服务的接口）

    -robot_state_class（状态类的目录）
        first_project_state.py （传球项目所需要的状态类）
        second_project_state.py （投篮项目所需要的状态类）

    -robot_state_pkg （和机器位置有关的接口的目录）
        get_robot_position.py （获取机器人位置信息的接口）

    control_state.py （总的运行文件，通关接收参数不同来运行不同的状态机文件）
    pass_ball_first.py
    pass_ball_second.py
    pass_ball_third.py
    shoot_ball_first.py
    shoot_ball_second.py
    shoot_ball_third.py

详细解释：



注意事项：



运行方法：
  传球项目：
	sudo chmod 0777 /dev/ttyUSB0 (打开串口)
        roslaunch basketball_bringup start_robot.launch （打开相关节点）
        rosrun basketball_catchone_srv findBall (打开寻找篮球的图像服务)	
	rosrun lineing findline (打开检测边线的图像服务)
	rosrun basketball_strage pass_ball_>>> （最后为具体的状态机文件，运行状态机）

  投篮项目：
        sudo chmod 0777 /dev/ttyUSB0 (打开串口)
	roslaunch basketball_bringup start_robot.launch （打开相关节点）
        rosrun volleyball_detect findvolleyball （打开寻找排球的图像服务）
	rosrun cylinder_detector findcylinder  （打开检测定位柱的图像服务）
	rosrun basketball_strage shoot_ball_>>> （最后为具体的状态机文件，运行状态机）

  注意：建议打开里程数据，检测陀螺仪数据是否正常
	rostopic echo /RecvData/1 

或者直接运行界面，通过界面点击运行。（未完成，部分功能未实现）
	rosrun rqt_control rqtControl.py

