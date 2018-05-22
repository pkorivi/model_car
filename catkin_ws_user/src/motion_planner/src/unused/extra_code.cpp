
/*Code for using the heuristic*/
//Create the trajectories till the forward most trajectory is of lowest cost and is evaluated.
while(final_states.front().evaluated != true){
  //TODO change to insertion sort - this vector is almost sorted
  //create_traj(final_states.front());
  double cost_val = create_traj_const_acc_xy_spline(current_vehicle_state,m_prev_vehicle_state,mp_traj_eval, final_states.front(),current_pos_map,curr_frenet_coordi);
  //std::cout <<" ID: "<<final_states.front().id <<" cost :  " << final_states.front().cost<< "  "<< final_states.front().evaluated<< '\n';
  ROS_INFO("Traj Eval ID: %d, cost %.2f cur v,s,d,th %.2f,%.2f,%.2f,%.2f ,x,y,yaw %.2f,%.2f,%.3f odom x,y %.3f,%.3f ,tgt a,v,s,d %.2f,%.2f,%.2f,%.2f !!",final_states.front().id,final_states.front().cost,vel_current,curr_frenet_coordi.s, \
  curr_frenet_coordi.d,curr_frenet_coordi.th,current_pos_map[0],current_pos_map[1],current_vehicle_state.getVehicleYaw(),current_vehicle_state.m_vehicle_position[0],current_vehicle_state.m_vehicle_position[1],final_states.front().a_tgt,final_states.front().v_tgt,final_states.front().s_tgt,final_states.front().d_eval );
  sort( final_states.begin(),final_states.end(), [ ](const target_state& ts1, const target_state& ts2){
        return ts1.cost < ts2.cost;});
}

/*Evaluate all the trajectories*/
for (size_t i = 0; i < final_states.size(); i++) {
  double cost_val = create_traj_const_acc_xy_spline(current_vehicle_state,m_prev_vehicle_state,mp_traj_eval, final_states[i],current_pos_map,curr_frenet_coordi);
  ROS_INFO("Traj Eval ID: %d, cost %.2f cur v,s,d,th %.2f,%.2f,%.2f,%.2f ,x,y,yaw %.2f,%.2f,%.3f odom x,y %.3f,%.3f ,tgt a,v,s,d %.2f,%.2f,%.2f,%.2f !!",final_states[i].id,final_states[i].cost,vel_current,curr_frenet_coordi.s, \
  curr_frenet_coordi.d,curr_frenet_coordi.th,current_pos_map[0],current_pos_map[1],current_vehicle_state.getVehicleYaw(),current_vehicle_state.m_vehicle_position[0],current_vehicle_state.m_vehicle_position[1],final_states[i].a_tgt,final_states[i].v_tgt,final_states[i].s_tgt,final_states[i].d_eval );
}


/*Sort and send the final trajectory out*/
sort( final_states.begin(),final_states.end(), [ ](const target_state& ts1, const target_state& ts2){
      return ts1.cost < ts2.cost;});
