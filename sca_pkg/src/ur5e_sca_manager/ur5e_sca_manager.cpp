#include "sca_pkg/ur5e_sca_manager/ur5e_sca_manager.hpp"

namespace rci_summit_ur5_controller
{
    SummitUR5_ScaManager::SummitUR5_ScaManager()
    {
        // Learning Weights Loading 
        std::string path_to_resPkg = ros::package::getPath("sca_pkg");
        std::string pkg_to_resFoldfer = "/result/result_txt";
        std::string path_to_res1_weight = "/l1_weight.txt";
        std::string path_to_res1_bias = "/l1_bias.txt";
        std::string path_to_res2_weight = "/l2_weight.txt";
        std::string path_to_res2_bias = "/l2_bias.txt";
        std::string path_to_res3_weight = "/l3_weight.txt";
        std::string path_to_res3_bias = "/l3_bias.txt";
        std::string path_to_res4_weight = "/l4_weight.txt";
        std::string path_to_res4_bias = "/l4_bias.txt";
        std::string path_to_res5_weight = "/l5_weight.txt";
        std::string path_to_res5_bias = "/l5_bias.txt";

        weight1_ = read_txt_to_matrix(path_to_resPkg + pkg_to_resFoldfer + path_to_res1_weight);
        bias1_ = read_txt_to_matrix(path_to_resPkg + pkg_to_resFoldfer + path_to_res1_bias);
    
        weight2_ = read_txt_to_matrix(path_to_resPkg + pkg_to_resFoldfer + path_to_res2_weight);
        bias2_ = read_txt_to_matrix(path_to_resPkg + pkg_to_resFoldfer + path_to_res2_bias);
    
        weight3_ = read_txt_to_matrix(path_to_resPkg + pkg_to_resFoldfer + path_to_res3_weight);
        bias3_ = read_txt_to_matrix(path_to_resPkg + pkg_to_resFoldfer + path_to_res3_bias);
    
        weight4_ = read_txt_to_matrix(path_to_resPkg + pkg_to_resFoldfer + path_to_res4_weight);
        bias4_ = read_txt_to_matrix(path_to_resPkg + pkg_to_resFoldfer + path_to_res4_bias);
    
        weight5_ = read_txt_to_matrix(path_to_resPkg + pkg_to_resFoldfer + path_to_res5_weight);
        bias5_ = read_txt_to_matrix(path_to_resPkg + pkg_to_resFoldfer + path_to_res5_bias);

        weights_ = {weight1_, weight2_, weight3_, weight4_, weight5_};
        biases_  = {bias1_, bias2_, bias3_, bias4_, bias5_};

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

    double SummitUR5_ScaManager::forward_pass(const Eigen::VectorXd& q)
    {
        Eigen::VectorXd x = q;
        for (size_t i = 0; i < weights_.size() - 1; ++i) {
            x = tanh(weights_[i] * x + biases_[i]);
        }
    
        Eigen::VectorXd out = weights_.back() * x + biases_.back();
        Eigen::VectorXd prob = softmax(out);
        
        return prob(0)-prob(1);
    }

    Eigen::MatrixXd SummitUR5_ScaManager::numerical_jacobian_gamma(const Eigen::VectorXd& q, double h)
    {
        const int n = q.size();
        const int m = 1;  // 또는 output 차원 추론 가능 시 q_output.size()
        Eigen::MatrixXd J(m, n);
    
        for (int i = 0; i < n; ++i) {
            Eigen::VectorXd dq = Eigen::VectorXd::Zero(n);
            dq(i) = h;
    
            double y_plus  = this->forward_pass(q + dq);
            double y_minus = this->forward_pass(q - dq);
    
            J(0,i) = (y_plus - y_minus) / (2.0 * h);
        }
    
        return J;
    }

}