<?xml version="1.0" ?>
<sdf version="1.10">
  <world name='maze_world'>
    <plugin
      filename="gz-sim-physics-system"
      name="gz::sim::systems::Physics">
    </plugin>

    <plugin
      filename="gz-sim-user-commands-system"
      name="gz::sim::systems::UserCommands">
    </plugin>

    <plugin
      filename="gz-sim-scene-broadcaster-system"
      name="gz::sim::systems::SceneBroadcaster">
    </plugin>


    <!-- Ground plane -->
    <model name="ground_plane">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <plane><normal>0 0 1</normal></plane>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <plane>
              <normal>0 0 1</normal>
              <size>60 60</size>
            </plane>
          </geometry>
          <material>
            <diffuse>1.0 1.0 1.0 1</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <model name='Maze_ng'>
    <static>true</static>
	<pose>0 0 0 0 0 0</pose>	<link name='Wall_0_0_0'>
		<pose>0 0 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_0_0_3'>
		<pose>0.5 -0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_0_1_0'>
		<pose>0 1 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_0_1_3'>
		<pose>0.5 0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_0_2_0'>
		<pose>0 2 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_0_3_0'>
		<pose>0 3 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_0_4_0'>
		<pose>0 4 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_0_5_0'>
		<pose>0 5 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_0_6_0'>
		<pose>0 6 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_0_6_1'>
		<pose>0.5 6.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_1_0_3'>
		<pose>1.5 -0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_1_1_0'>
		<pose>1 1 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_1_2_3'>
		<pose>1.5 1.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_1_3_0'>
		<pose>1 3 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_1_3_3'>
		<pose>1.5 2.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_1_4_0'>
		<pose>1 4 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_1_5_0'>
		<pose>1 5 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_1_6_0'>
		<pose>1 6 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_1_6_1'>
		<pose>1.5 6.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_2_0_0'>
		<pose>2 0 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_2_0_3'>
		<pose>2.5 -0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_2_1_3'>
		<pose>2.5 0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_2_2_3'>
		<pose>2.5 1.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_2_4_0'>
		<pose>2 4 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_2_5_0'>
		<pose>2 5 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_2_5_3'>
		<pose>2.5 4.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_2_6_3'>
		<pose>2.5 5.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_2_6_1'>
		<pose>2.5 6.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_3_0_3'>
		<pose>3.5 -0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_3_1_3'>
		<pose>3.5 0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_3_2_0'>
		<pose>3 2 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_3_2_3'>
		<pose>3.5 1.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_3_3_0'>
		<pose>3 3 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_3_4_3'>
		<pose>3.5 3.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_3_6_3'>
		<pose>3.5 5.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_3_6_1'>
		<pose>3.5 6.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_4_0_3'>
		<pose>4.5 -0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_4_1_3'>
		<pose>4.5 0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_4_2_3'>
		<pose>4.5 1.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_4_3_3'>
		<pose>4.5 2.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_4_4_0'>
		<pose>4 4 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_4_4_3'>
		<pose>4.5 3.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_4_5_0'>
		<pose>4 5 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_4_6_1'>
		<pose>4.5 6.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_5_0_3'>
		<pose>5.5 -0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_5_2_3'>
		<pose>5.5 1.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_5_3_3'>
		<pose>5.5 2.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_5_4_3'>
		<pose>5.5 3.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_5_5_0'>
		<pose>5 5 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_5_5_3'>
		<pose>5.5 4.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_5_6_3'>
		<pose>5.5 5.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_5_6_1'>
		<pose>5.5 6.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_0_3'>
		<pose>6.5 -0.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_0_2'>
		<pose>7 0 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_1_0'>
		<pose>6 1 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_1_2'>
		<pose>7 1 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_2_2'>
		<pose>7 2 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_3_3'>
		<pose>6.5 2.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_3_2'>
		<pose>7 3 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_4_2'>
		<pose>7 4 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_5_3'>
		<pose>6.5 4.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_5_2'>
		<pose>7 5 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_6_2'>
		<pose>7 6 0 0 0 -1.570795</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	<link name='Wall_6_6_1'>
		<pose>6.5 6.5 0 0 0 0</pose>
		<self_collide>0</self_collide>
		<kinematic>0</kinematic>
		<visual name='Wall_Visual'>
			<pose>0 0 0 0 0 0 </pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
			<cast_shadows>1</cast_shadows>
			<material><diffuse>0.7 0.7 0.7 1</diffuse></material>
		</visual>
		<collision name='Wall_Collision'>
			<laser_retro>0</laser_retro>
			<max_contacts>10</max_contacts>
			<pose>0 0 0 0 -0 0</pose>
			<geometry>
				<box>
					<size>1 0.1 1</size>
				</box>
			</geometry>
		</collision>
	</link>
	</model>
	</world>
</sdf>