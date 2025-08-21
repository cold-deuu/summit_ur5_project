#include "sca_pkg/main.hpp"

int main(int argc, char ** argv)
{
    // ROS
    ros::init(argc, argv, "data_collector");
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

    // Joint Value
    q_.resize(model.nq);
    v_.resize(model.nv);
    v_.setZero();

    // Franka Joint Limit
    qMin_ = (Eigen::VectorXd(6) <<-3.141592, -3.141592,-3.141592,-3.141592,-3.141592,-3.141592).finished();
    qMax_ = (Eigen::VectorXd(6) << 3.141592, 3.141592, 3.141592, 3.141592, 3.141592, 3.141592).finished();
    
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
	c_req.enable_contact = true;
	d_req.enable_signed_distance = true;
	d_req.enable_nearest_points = true;

    // About Data
    int nSamples = 500000;
    double fracCollision = 0.5;
    double fracFree = 0.15;
    double fracThreshold = 0.35;
    if (std::abs(fracCollision + fracFree + fracThreshold - 1.0) > 1e-8) {
        double sum = fracCollision + fracFree + fracThreshold;
        fracCollision /= sum;
        fracFree /= sum;
        fracThreshold /= sum;
    }
    int nCollision = nSamples * fracCollision;
    int nFree = nSamples * fracFree;
    int nThreshold = nSamples * fracThreshold;
    int nTotal = nCollision + nFree + nThreshold;
    Eigen::VectorXd line(model.nq-3 + 1);
    Eigen::VectorXd qRand(model.nq-3), qRandNormal(model.nq-3);
    std::vector<Eigen::VectorXd> samplingData;

    // For Sampling
    bool collisionFlag = true;
    bool stableFlag, thresFlag;
    double thresholdDist = 0.2;
    double collisionDist = 0.03;
    int current_collisions = 0;
    int current_non_collisions = 0;
    double thresholdDist2 = 0.08;


    // TEST
    int testCol = 0;
    int testThres = 0;
    int testFree = 0;

    int iter = 0;
    while(iter<nTotal)
    {
        // Joint Sampling
        for(int i=0; i<model.nq-3; i++)
        {
            qRandNormal(i) = unif(rng);
            qRand(i) = qMin_(i) + qRandNormal(i) * (qMax_(i) - qMin_(i));
        }
        line.setZero();
        line.segment(1,model.nq-3) = qRand;
        Eigen::VectorXd mm_qrand(model.nq);
        mm_qrand.setZero();
        mm_qrand.tail(model.nq-3) = qRand;
        pinocchio::computeAllTerms(model, data, mm_qrand, v_);        
        std::vector<pinocchio::SE3> linkPoseSet = computeAllPose(model, data, jointName_idx_map);
        updateCollisionObj(colObjs, linkPoseSet, joint_idx_map);


        bool prevFlag = collisionFlag;
        collisionFlag = false;


        for(int i=0; i<joint_idx_map.size(); i++)
        {
            for(int j=0; j<collision_grid[i].size(); j++)
            {
                int link1 = i;
                int link2 = collision_grid[i][j];
                c_res.clear();
                fcl::collide(colObjs[link1], colObjs[link2], c_req, c_res);

                if(c_res.isCollision())
                {
                    collisionFlag = true;
                    break;
                }
            }
            if(collisionFlag) break;
        }


        thresFlag = true;
        if(!collisionFlag)
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
    

                    if (link1 ==0 && link2 ==3)
                    {
                        if(d_res.min_distance < thresholdDist2 && d_res.min_distance > collisionDist)
                        {
                            thresFlag = true;
                        }
                        if(d_res.min_distance < collisionDist)
                        {
                            collisionFlag = true;
                            break;
                        }
                    }
                    else{
                        if(d_res.min_distance < thresholdDist && d_res.min_distance > collisionDist)
                        {
                            thresFlag = true;
                        }
                        if(d_res.min_distance < collisionDist)
                        {
                            collisionFlag = true;
                            break;
                        }
                    }

                }
            if(collisionFlag) break;
            }
        }
        if(collisionFlag && current_collisions < nTotal/2)
        {
            line(0) = 1.0;
            samplingData.push_back(line);
            current_collisions ++;
            iter++;

            //TEST
            testCol ++;
        }
        if(!collisionFlag && current_non_collisions < nTotal/2)
        {

            if(nThreshold==0 && nFree > 0.5 * (nTotal/2 - nTotal/2*fracThreshold))
            {
                thresFlag = true;
                nThreshold++;
            }

            if(thresFlag && nThreshold>0)
            {
                nThreshold = nThreshold-1;
                samplingData.push_back(line);
                current_non_collisions ++;
                iter++;


                // TEST
                testThres ++;
            }

            if(!thresFlag && nFree > 0)
            {
                nFree --;
                samplingData.push_back(line);
                current_non_collisions ++;
                iter ++;
                
                //TEST
                testFree++;
            }


        }

        if(iter % 100 == 0)
        {
            double sampling_percent = double(iter) / double(nTotal) * 100.0;
            std::cout << std::fixed << std::setprecision(2);  // 소수점 둘째 자리까지
            std::cout << "Sampling Percent : " << sampling_percent << "%" << std::endl;
        }

        std::cout<<"Collisions : "<<testCol<<std::endl;
        std::cout<<"Thres : "<<testThres<<std::endl;
        std::cout<<"Free : "<<testFree<<std::endl;
    }

    // std::cout<<"Collisions : "<<testCol<<std::endl;
    // std::cout<<"Thres : "<<testThres<<std::endl;
    // std::cout<<"Free : "<<testFree<<std::endl;


    
    int res_size = samplingData.size();
    std::cout<<"Result Size : "<<res_size<<std::endl;
    std::cout<<"Collisions : "<<testCol<<std::endl;
    std::cout<<"Thres : "<<testThres<<std::endl;
    std::cout<<"Free : "<<testFree<<std::endl;


    // // Data Save
    // std::string package_to_data = "/src/data/data_";
    // std::string data_path = mesh_package_path + package_to_data;
    // std::string bin_fname = data_path+std::to_string(nTotal/1000)+".bin";
	// auto file = std::fstream(bin_fname, std::ios::out | std::ios::binary);
	// file.write((char*) &res_size, sizeof(int));	
	// int lsize = samplingData[0].size();
	// file.write((char*) &lsize, sizeof(int));	
	// for(int i=0;i<res_size;i++)
	// {	
	// 	for(int j=0;j<lsize;j++)
	// 	{				
	// 		float d_f = (float) samplingData[i][j];
	// 		file.write((char*) &d_f, sizeof(float));
	// 	}
	// }
    // file.close();

    // Data Save to TXT
    std::string package_to_data = "/src/data/data_";
    std::string data_path = sca_pkg_path + package_to_data;
    std::string txt_fname = data_path + std::to_string(nTotal/1000) + ".txt";

    std::ofstream file(txt_fname);  // 텍스트 출력 스트림

    if (!file.is_open()) {
        std::cerr << "Failed to open file for writing: " << txt_fname << std::endl;
    } else {
        int res_size = samplingData.size();
        int lsize = samplingData[0].size();

        file << res_size << " " << lsize << "\n";  // 첫 줄에 크기 정보

        for (int i = 0; i < res_size; ++i) {
            for (int j = 0; j < lsize; ++j) {
                file << samplingData[i][j];
                if (j != lsize - 1) file << " ";
            }
            file << "\n";
        }

        file.close();
        std::cout << "Saved data to " << txt_fname << std::endl;
    }

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