#include "sca_pkg/ur5e_sca_manager/ur5e_sca_manager.hpp"

namespace rci_summit_ur5_controller
{
    SummitUR5_ScaManager::SummitUR5_ScaManager()
    {
        // Learning Weights Loading 
        std::string path_to_resPkg = ros::package::getPath("sca_pkg");
        std::string pkg_to_folder_mobile = "/result/mobile/result_txt";
        std::string pkg_to_folder_manip = "/result/manip/result_txt";
        std::string path_to_res1_weight = "/l1_weight.txt";
        std::string path_to_res1_bias = "/l1_bias.txt";
        std::string path_to_res2_weight = "/l2_weight.txt";
        std::string path_to_res2_bias = "/l2_bias.txt";
        std::string path_to_res3_weight = "/l3_weight.txt";
        std::string path_to_res3_bias = "/l3_bias.txt";
        std::string path_to_res4_weight = "/l4_weight.txt";
        std::string path_to_res4_bias = "/l4_bias.txt";


        // Load Weights : Hard Coded
        mobile_weight1_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_mobile + path_to_res1_weight);
        mobile_bias1_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_mobile + path_to_res1_bias);
        mobile_weight2_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_mobile + path_to_res2_weight);
        mobile_bias2_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_mobile + path_to_res2_bias);
        mobile_weight3_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_mobile + path_to_res3_weight);
        mobile_bias3_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_mobile + path_to_res3_bias);
        mobile_weight4_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_mobile + path_to_res4_weight);
        mobile_bias4_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_mobile + path_to_res4_bias);

        manip_weight1_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_manip + path_to_res1_weight);
        manip_bias1_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_manip + path_to_res1_bias);
        manip_weight2_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_manip + path_to_res2_weight);
        manip_bias2_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_manip + path_to_res2_bias);
        manip_weight3_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_manip + path_to_res3_weight);
        manip_bias3_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_manip + path_to_res3_bias);
        manip_weight4_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_manip + path_to_res4_weight);
        manip_bias4_ = read_txt_to_matrix(path_to_resPkg + pkg_to_folder_manip + path_to_res4_bias);

        // Mobile <-> Manipulator
        mobile_weights_ = {mobile_weight1_, mobile_weight2_, mobile_weight3_, mobile_weight4_};
        mobile_biases_  = {mobile_bias1_, mobile_bias2_, mobile_bias3_, mobile_bias4_};

        // Manipulator itself
        manip_weights_ = {manip_weight1_, manip_weight2_, manip_weight3_, manip_weight4_};
        manip_biases_  = {manip_bias1_, manip_bias2_, manip_bias3_, manip_bias4_};

    };

    Eigen::MatrixXd SummitUR5_ScaManager::read_txt_to_matrix(const std::string& filename) {
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
    
    // Utils
    Eigen::VectorXd SummitUR5_ScaManager::tanh(const Eigen::VectorXd& x) {
        return x.array().tanh();
    }

    Eigen::VectorXd SummitUR5_ScaManager::softmax(const Eigen::VectorXd& x) {
        Eigen::ArrayXd exps = (x.array() - x.maxCoeff()).exp();
        return (exps / exps.sum()).matrix();
    }

    std::map<std::string, double> SummitUR5_ScaManager::forward_pass(const Eigen::VectorXd& q)
    {
        // Manip
        Eigen::VectorXd x_manip = q;
        for (size_t i = 0; i < manip_weights_.size() - 1; ++i) {
            x_manip = tanh(manip_weights_[i] * x_manip + manip_biases_[i]);
        }
    
        Eigen::VectorXd out_manip = manip_weights_.back() * x_manip + manip_biases_.back();
        Eigen::VectorXd prob_manip = softmax(out_manip);

        double gamma_manip = prob_manip(0) - prob_manip(1);

        // Mobile
        Eigen::VectorXd x_mobile = q;
        for (size_t i = 0; i < mobile_weights_.size() - 1; ++i) {
            x_mobile = tanh(mobile_weights_[i] * x_mobile + mobile_biases_[i]);
        }
    
        Eigen::VectorXd out_mobile = mobile_weights_.back() * x_mobile + mobile_biases_.back();
        Eigen::VectorXd prob_mobile = softmax(out_mobile);

        double gamma_mobile = prob_mobile(0) - prob_mobile(1);

        std::map<std::string, double> gamma;

        gamma["mobile"] = gamma_mobile;
        gamma["manip"] = gamma_manip;

        return gamma;
    }

    std::map<std::string, Eigen::MatrixXd> SummitUR5_ScaManager::numerical_jacobian_gamma(const Eigen::VectorXd& q, double h)
    {
        const int n = q.size();
        const int m = 1;  // 또는 output 차원 추론 가능 시 q_output.size()
        Eigen::MatrixXd J_mobile(m, n);
        Eigen::MatrixXd J_manip(m, n);
        
    
        for (int i = 0; i < n; ++i) {
            Eigen::VectorXd dq = Eigen::VectorXd::Zero(n);
            dq(i) = h;
    
            auto y_plus  = this->forward_pass(q + dq);
            auto y_minus = this->forward_pass(q - dq);
        
            J_mobile(0,i) = (y_plus["mobile"] - y_minus["mobile"]) / (2.0 * h);
            J_manip(0,i) = (y_plus["manip"] - y_minus["manip"]) / (2.0 * h);
        }
        
        std::map<std::string, Eigen::MatrixXd> J;
        J["mobile"] = J_mobile;
        J["manip"] = J_manip;

        return J;
    }

}