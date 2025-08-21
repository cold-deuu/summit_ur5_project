#include "sca_pkg/meshes/mesh_utils.hpp"

using namespace std;
using namespace Eigen;
namespace fs = std::filesystem;

std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (first == std::string::npos) return "";
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, last - first + 1);
}

Body read_txt_meshes(const string & path)
{
    //Getting the list of filenames with meshes - faces and points
    vector <string> mesh_fnames;
    ostringstream tmp;
    //string path = "meshes/txts";
    for (auto & p : fs::directory_iterator(path))
    {
        tmp.str("");
        tmp << p;
        string str_tmp = tmp.str();
        mesh_fnames.push_back(str_tmp.substr(1,str_tmp.size()-2));
    }
    std::cout<<"Finish read all meshes of franka"<<std::endl;
    //Moving points and faces from strings in files to a single vector meshes
    //vector of meshes, each has id (int), vertices(3x double) and faces (3x int)
    Body meshes(mesh_fnames.size());
    cout<<"Size of Links :"<<mesh_fnames.size()<<endl;
    for (int i=0;i<mesh_fnames.size();i++)
    {
        std::cout<<"Start create "<<i<<"-th link : "<<mesh_fnames[i]<<std::endl;
        vector<Vector3d> Vertices;
        vector<Vector3i> Faces;
        ifstream file(mesh_fnames[i].c_str());
        string line;
        getline(file, line); //Read index of body
        
        int idx = stoi(line);
        getline(file, line); //Read body name
        string bname = line;
        getline(file, line); //Read "V"
        string V = line;
        getline(file, line); //Read first string of three floats
        int itera = 0;
        //Reading the Vertices first
        while (true) 
        {   

            // std::replace(line.begin(), line.end(), '\t', ' ');
            // line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
            // line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
            stringstream ss(line);
            double single_vec[3];
            for(int j=0;j<3;j++)
            {
                ss >> single_vec[j];
            }

            Vertices.push_back(Vector3d(single_vec[0],single_vec[1],single_vec[2]));
            // cout << Vertices.back().transpose() << endl;
            //printf("%s \n", line.c_str());
            getline(file, line);
            if (trim(line) == "F")
            {
                break;
            }
        } 
            
        while (getline(file, line)) //parsing faces of the mesh
        {   
            stringstream ss(line);
            int single_vec[3];
            for(int j=0;j<3;j++)
            {
                ss >> single_vec[j];
            }
            Faces.push_back(Vector3i(single_vec[0]-1,single_vec[1]-1,single_vec[2]-1));
            //cout << Faces.back().transpose() << endl;
            //printf("%s \n", line.c_str());
        } 
        file.close();
        meshes[idx] = make_tuple(idx, Vertices, Faces, bname);
        // meshes.push_back(make_tuple(idx, Vertices, Faces, bname));
        std::cout<<"Finish create "<<i<<"-th link"<<std::endl;
    }
    return meshes;
}

int write_txt_meshes(Body &meshes, string & fname_out)
{
    for(int i=0;i<meshes.size();i++)
    {
        ofstream f_id;
        string fname = fname_out + to_string(get<0>(meshes[i])) + ".txt";
        f_id.open(fname);
        f_id << get<0>(meshes[i]) << "\n" << get<3>(meshes[i]) << "\n" << "V" << "\n";
        auto V = get<1>(meshes[i]);
        for(int j=0;j<V.size();j++)
        {
            f_id << V[j].transpose() << "\n";
        }
        f_id << "F\n";
        auto F = get<2>(meshes[i]);
        for(int j=0;j<F.size();j++)
        {
            f_id << F[j].transpose() + Vector3i(1,1,1).transpose() << "\n";
        }        
        f_id.close();
    }
    return 0;
}

Body update_mesh(const Body &meshes_dflt, std::vector<pinocchio::SE3> linkPoseSet)
{
	int N_bodies = meshes_dflt.size();
	Body meshes_new;
	for(int i=0;i<N_bodies;i++)
	{
		int body_idx = get<0>(meshes_dflt[i]);
        Matrix3d R_mat = linkPoseSet[i].rotation();
		Vector3d T_vec = linkPoseSet[i].translation();
		//cout << R_mat << "\n" << T_vec.transpose() << "\n\n";
		auto p_cloud_init = get<1>(meshes_dflt[i]);
		vector<Vector3d> p_cloud_new;
		for(int j=0;j<p_cloud_init.size();j++)
		{
			p_cloud_new.push_back(R_mat * p_cloud_init[j] + T_vec);
		}
		meshes_new.push_back(make_tuple(body_idx,p_cloud_new,get<2>(meshes_dflt[i]),get<3>(meshes_dflt[i])));
	}
    return meshes_new;
}


vector<fcl::Triangle> createTriangle(const vector<Vector3i> faces)
{
    vector<fcl::Triangle> trs;
    for(int i=0;i<faces.size();i++)
    {
        trs.push_back(fcl::Triangle(faces[i][0],faces[i][1],faces[i][2]));
    }
    return trs;
}

vector<fcl::CollisionObjectd*> createCollisionObj(const Body &meshes)
{
	int N_bodies = meshes.size();
    vector<fcl::CollisionObjectd*> res(N_bodies);
    for(int i=0;i<N_bodies;i++)
    {
        int id = get<0>(meshes[i]); // 몇번째 링크인지
        std::shared_ptr<fcl_body> geom = std::make_shared<fcl_body>(); // 
        geom->beginModel();
        geom->addSubModel(get<1>(meshes[i]),createTriangle(get<2>(meshes[i])));
        geom->endModel();
        res[i] = new fcl::CollisionObjectd(geom);
    }
    return res;
}

void updateCollisionObj(std::vector<fcl::CollisionObjectd*> colObjs, std::vector<pinocchio::SE3> linkPoseSet, VectorXi &idx_map)
{
	int N_bodies = colObjs.size();

    for(int i=0;i<N_bodies;i++)
    {	
        Matrix3d R_mat = linkPoseSet[i].rotation();
		Vector3d T_vec = linkPoseSet[i].translation();
        

        //cout << i<<": " << T_vec.transpose() << endl << R_mat << endl;
        colObjs[i]->setRotation(R_mat);    
        colObjs[i]->setTranslation(T_vec);



    }
}


vector<VectorXi> read_noncol_map(const string& fname)
{       
    vector<VectorXi> res; 
    ifstream file(fname.c_str()); 
    string line;       

    while (getline(file, line)) 
    {   
        istringstream ss(line);
        vector<int> std_vec;
        std_vec = vector<int>(istream_iterator<int>(ss),std::istream_iterator<int>());
        VectorXi e_vec(std_vec.size());
        for(int i = 0;i<std_vec.size();i++)
        {
            e_vec[i] = std_vec[i];
        }
        res.push_back(e_vec);
    } 
    file.close();
    return res;
}

int find_idx(const VectorXi & array, const int & idx)
{
    for(int i = 0;i<array.size();i++)
    {
        if(array[i] == idx){
            return i;
        }
    }
}

vector<VectorXi> CollisionsBodyGrid(const Body& meshes, vector<VectorXi> collision_map, VectorXi idx_map)
{
    int n_bodies = meshes.size();
    vector<VectorXi> res_grid(n_bodies);
    //cout << "Collision body grid: " << endl;
    for(int i=0;i<n_bodies;i++)
    {   
        int c = 0;
        int i1 = get<0>(meshes[i]); // 몇번째 링크?, int
        cout<<"Link1 :"<<i1<<endl;
        for(int j=i+1;j<n_bodies;j++)
        {
            int i2 = get<0>(meshes[j]); // i2 는 i1 보다 큰 인덱스의 링크
            
            for(int k=0;k<collision_map.size();k++)
            {   
                if((collision_map[k][0] == i1) && (collision_map[k].tail(collision_map[k].size()-1).array() == i2).any() )
                // 만약 collision Map에서 Collision 할 수 있다고 판단되면
                {
                    res_grid[i].conservativeResize(c+1);
                    res_grid[i][c] = j;
                    c+=1;
                }
            }
        }
        //cout << res_grid[i].transpose() << endl;
    }
    return res_grid;
}

vector<VectorXf> read_bin_data(const std::string& fname)
{
    auto file = fstream(fname, ios::in | ios::binary);
    if (file.is_open())
        cout<< "File is opened!" << endl;

    int data_size, entry_size;

    file.read((char*)&data_size,sizeof(int));
    file.read((char*)&entry_size,sizeof(int));

    cout << "Dataset size is " << data_size << " x " << entry_size << endl;
    vector<VectorXf> data;
    data.resize(data_size);
    for(int i = 0; i < data.size(); i++)
    {   
        data[i].resize(entry_size);
        for(int j = 0; j < data[i].size(); j++)
        {
            file.read((char*)&data[i][j],sizeof(float));
        }
    }
    file.close();
    return data;
}

Eigen::MatrixXd read_txt_to_matrix(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filename);
    }

    std::vector<std::vector<float>> data;
    std::string line;

    // 먼저 파일 전체 라인 읽기
    while (std::getline(file, line)) {
        std::stringstream ss(line);
        std::vector<float> row;
        float value;
        while (ss >> value) {
            row.push_back(value);
        }
        if (!row.empty()) {
            data.push_back(row);
        }
    }

    file.close();

    // 행/열 자동 파악
    const size_t rows = data.size();
    const size_t cols = data[0].size();
    Eigen::MatrixXd mat(rows, cols);

    // 값 복사
    for (size_t i = 0; i < rows; ++i) {
        if (data[i].size() != cols) {
            throw std::runtime_error("Inconsistent column size in row " + std::to_string(i));
        }
        for (size_t j = 0; j < cols; ++j) {
            mat(i, j) = data[i][j];
        }
    }

    return mat;
}


vector<VectorXf> read_txt_data(const std::string& fname) {
    auto file = fstream(fname, ios::in);
    if (file.is_open())
        cout << "File is opened!" << endl;

    int data_size, entry_size;
    file >> data_size >> entry_size;

    cout << "Dataset size is " << data_size << " x " << entry_size << endl;
    vector<VectorXf> data;
    data.resize(data_size);
    for (int i = 0; i < data_size; i++) {
        data[i].resize(entry_size);
        for (int j = 0; j < entry_size; j++) {
            file >> data[i][j];
        }
    }
    file.close();
    return data;
}



