#include "sca_pkg/fcl_tutorial.hpp"
// activation functions
Eigen::VectorXd tanh(const Eigen::VectorXd& x) {
    return x.array().tanh();
}
Eigen::VectorXd softmax(const Eigen::VectorXd& x) {
    Eigen::ArrayXd exps = (x.array() - x.maxCoeff()).exp();
    return (exps / exps.sum()).matrix();
}
Eigen::MatrixXd numerical_jacobian_gamma(
    const std::function<Eigen::Vector2d(const Eigen::VectorXd&)>& gamma,
    const Eigen::VectorXd& q,
    double h = 1e-6)
{
    const int n = q.size();
    Eigen::MatrixXd J(2, n);  // 2xN Jacobian

    for (int i = 0; i < n; ++i) {
        Eigen::VectorXd dq = Eigen::VectorXd::Zero(n);
        dq(i) = h;

        Eigen::Vector2d gamma_plus  = gamma(q + dq);
        Eigen::Vector2d gamma_minus = gamma(q - dq);

        J.col(i) = (gamma_plus - gamma_minus) / (2.0 * h);
    }

    return J;
}
int main(int argc, char ** argv)
{
    // ROS
    ros::init(argc, argv, "fcl_tester");
    ros::NodeHandle nh;

    // RVIZ Joint Subscriber
    ros::Subscriber sub = nh.subscribe("/joint_states", 1000, rvizCallBack);

    // Timer
    ros::Rate loop_rate(1000);

    // Generate Random Seed 
    std::mt19937_64 rng;
    uint64_t timeSeed = std::chrono::high_resolution_clock::now().time_since_epoch().count();
    std::seed_seq ss{uint32_t(timeSeed & 0xffffffff), uint32_t(timeSeed>>32)};
    rng.seed(ss);
    std::uniform_real_distribution<double> unif(0, 1);

    // Build Robot
    std::string urdf_package_path = ros::package::getPath("summit_xl_description");
    std::string package_to_urdf = "/summit_ur_control.urdf";
    std::string urdf_path = urdf_package_path + package_to_urdf;

    pinocchio::Model model;
    pinocchio::urdf::buildModel(urdf_path, model);
    pinocchio::Model::Index joint_id;
    pinocchio::Data data(model);

    std::cout<<"NQ :"<<model.nq<<std::endl;

    // Joint Value
    q_.resize(model.nq);
    v_.resize(model.nv);
    q_.setZero();
    v_.setZero();

    // Import Meshes
    std::string mesh_package_path = ros::package::getPath("ur_e_description");
    std::string package_to_mesh = "/sca_meshes";
    std::string mesh_path = mesh_package_path + package_to_mesh;
    Body mesh_zero = read_txt_meshes(mesh_path);
    Eigen::VectorXi joint_idx_map(mesh_zero.size());  
    std::vector<std::string> name_idx_map(mesh_zero.size());
    std::vector<std::string> jointName_idx_map = {"summit_joint","base_joint", "ur5e_shoulder_pan_joint", 
                                                    "ur5e_shoulder_lift_joint", "ur5e_elbow_joint",
                                                    "ur5e_wrist_1_joint", "ur5e_wrist_2_joint", "ur5e_wrist_3_joint"};
    std::cout<<"Mesh Check"<<std::endl;
    for(int i=0; i<mesh_zero.size(); i++)
    {
        joint_idx_map(i) = std::get<0>(mesh_zero[i]);
        name_idx_map[i] = std::get<3>(mesh_zero[i]);

        std::cout<<i<<"-th mesh"<<std::endl;
        std::cout<<"Joint Index : "<<joint_idx_map(i)<<std::endl;
        std::cout<<"Name Index : "<<name_idx_map[i]<<std::endl;
    }

    // Get Self-Collision Map 
    std::string sca_pkg_path = ros::package::getPath("sca_pkg");
    std::string package_to_collisionIdx = "/src/meshes/collision_map/mm_collision_map.txt";
    std::string collisionIdx_path = sca_pkg_path + package_to_collisionIdx;
    auto collisionMap = read_noncol_map(collisionIdx_path);

    // Generate Collision Object
    std::vector<fcl::CollisionObjectd*> colObjs(mesh_zero.size());
    colObjs = createCollisionObj(mesh_zero);

    // Generate Collision Grid 
    std::vector<Eigen::VectorXi> collision_grid = CollisionsBodyGrid(mesh_zero, collisionMap, joint_idx_map);
    std::cout<<collision_grid.size()<<std::endl;
    for (int i=0; i<collision_grid.size();i++)
    {
        auto tmp = collision_grid[i];
        std::cout<<tmp.transpose()<<std::endl;
    }
    // FCL Library
    fcl::CollisionRequestd c_req;
	fcl::CollisionResultd c_res;
	fcl::DistanceRequestd d_req;
	fcl::DistanceResultd d_res;
	d_req.enable_signed_distance = true;
	d_req.enable_nearest_points = true;    
	c_req.enable_contact = true;
    bool thresFlag;
    double thresholdDist = 0.15;
    double collisionDist = 0.03;

    double thresholdDist2 = 0.08;


    

    int iter = 0;
    std::vector<pinocchio::SE3> linkPoseSet;
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
        // q_<< -2.9, -0.59, 2.9, -3.07, -2.9, 1.43, 1.48;
        bool flag = false;

        pinocchio::computeAllTerms(model, data, q_, v_);        
        linkPoseSet = computeAllPose(model, data, jointName_idx_map);
        updateCollisionObj(colObjs, linkPoseSet, joint_idx_map);
        // std::cout<<"Here "<<std::endl;
        // auto mesh_new = update_mesh(mesh_zero, linkPoseSet);
        // colObjs = createCollisionObj(mesh_new);

        // std::cout<<q_.transpose()<<std::endl;
        for(int i = 0; i<joint_idx_map.size();i++)
        {
            for (int j=0; j<collision_grid[i].size(); j++)
            {
                int link1 = i;
                int link2 = collision_grid[i][j];
                c_res.clear();
                fcl::collide(colObjs[link1], colObjs[link2], c_req, c_res);
                if(c_res.isCollision()){
                    // std::cout << i <<"-th link and"<< link2 << "-th link Collision!" << std::endl;
                    ROS_ERROR_STREAM("FCL : Collision");
                    flag = true;
                    auto contact = c_res.getContact(0);
                    // std::cout << "Contact position: " << contact.pos.transpose() << std::endl;
                    // std::cout << "Penetration depth: " << contact.penetration_depth << std::endl;                    
                }                  
                // else std::cout<<"Stable"<<std::endl;
            }
           
            
        }
        thresFlag = true;

        if(!flag)
        {
            thresFlag = false;
            for(int i=0; i<joint_idx_map.size(); i++)
            {
                for(int j=0; j<collision_grid[i].size(); j++)
                {
                    int link1 = i;
                    int link2 = collision_grid[i][j];
                    d_res.clear();
                    fcl::distance(colObjs[link1], colObjs[link2], d_req, d_res);
                    // std::cout<<"Min Dist :"<<d_res.min_distance<<std::endl;
                    if(link1 == 0 && link2 ==3)
                    {
                        if(d_res.min_distance < thresholdDist2 && d_res.min_distance > collisionDist)
                        {
                            thresFlag = true;
                            std::cout << i <<"-th link and"<< link2 << "-th link Thres!" << std::endl;
                            // ROS_WARN_STREAM("FCL : Collision base and link2");
    
                        }
                        if(d_res.min_distance < collisionDist)
                        {
                            // ROS_ERROR_STREAM("FCL : Collision base and link2");
                            
                            std::cout << i <<"-th link and"<< link2 << "-th link Collision!" << std::endl;
                            flag = true;
                        }
                    }
                    else{
                        if(d_res.min_distance < thresholdDist && d_res.min_distance > collisionDist)
                        {
                            thresFlag = true;
                            std::cout << i <<"-th link and"<< link2 << "-th link Thres!" << std::endl;
                            // ROS_WARN_STREAM("FCL : Collision");
    
                        }
                        if(d_res.min_distance < collisionDist)
                        {
                            // ROS_ERROR_STREAM("FCL : Collision");
                            
                            std::cout << i <<"-th link and"<< link2 << "-th link Collision!" << std::endl;
                            flag = true;
                        }
                    }

            }
           }
        }
        if (!flag && !thresFlag) std::cout<<"Stable"<<std::endl;

        // auto mesh_new = update_mesh(mesh_zero, linkPoseSet);
        // std::string newMesh_path = sca_pkg_path + "/new_mesh/new_mesh";
        // write_txt_meshes(mesh_new, newMesh_path);
        // iter++;
        // if(iter >2000) break;
    }
    auto mesh_new = update_mesh(mesh_zero, linkPoseSet);
    std::string newMesh_path = sca_pkg_path + "/new_mesh/new_mesh";
    write_txt_meshes(mesh_new, newMesh_path);



}

void rvizCallBack(const sensor_msgs::JointState::ConstPtr& msg)
{
    if (q_.size()==0){
        ROS_INFO_STREAM("Waiting Initialize");
    }
    else{
        for(int i=0; i<q_.size()-3; i++)
        {
            q_(i+3) = msg->position[i];
        }
    }
    
}

std::vector<pinocchio::SE3> computeAllPose(const pinocchio::Model model, pinocchio::Data data, std::vector<std::string> fname_index)
{
    std::vector<pinocchio::SE3> linkPoseSet;
    
    for (int i = 0; i<fname_index.size(); i++)
    {
        pinocchio::SE3 linkPose;
        std::string jointName = fname_index[i];
        if (i==0)
        {
            linkPose.setIdentity();
        }

        else if (i==1)
        {
            linkPose.setIdentity();
            linkPose.translation() << 0.225, 0.0, 0.395;
        }

        else
        {
            linkPose = data.oMi[model.getJointId(jointName)];

        }
        linkPoseSet.push_back(linkPose);
        // std::cout<<i<<"-th link Pose"<<std::endl;
        // std::cout<<linkPose<<std::endl;        
    }

    return linkPoseSet;
}