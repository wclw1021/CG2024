#include "mix.h"
#include "cloning.h"
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <iostream>

namespace USTC_CG
{
using uchar = unsigned char;
void Mix::depict(std::shared_ptr<USTC_CG::Image> mask)
{
            // find the component on the mask
            std::vector<std::pair<int,int>> loca;
            for (int i = 0; i < mask->width(); ++i)
            {
                for (int j = 0; j < mask->height(); ++j)
                {
                    if (mask->get_pixel(i,j)[0] > 0)
                    {
                        loca.push_back({i,j});
                    }
                }
            }
            // give points on the mask id
            std::vector<std::vector<int>> id (mask->width(), std::vector<int>(mask->height(), -1));
            int num = static_cast<int>(loca.size());
            for (int i = 0; i < num; i++) 
            {
                auto [x, y] = loca[i];
                id[x][y] = i;
            }
            
            // make the Sparse Matrix 
            const std::vector<std::pair<int, int>> mov = {{-1, 0}, {1, 0}, {0, -1}, {0, 1}};
            Eigen::SparseMatrix<double> A(num , num);
            std::vector<Eigen::MatrixXd> b(data_->channels(),Eigen::MatrixXd::Zero(num, 1));
            for (int i = 0; i < num; ++i)
            {
                auto [x, y] = loca[i];
                for (auto [dx, dy] : mov) 
                {
                    int nx = x + dx;
                    int ny = y + dy;
                    if (nx < 0 || nx >= mask->width() || ny < 0 || ny >= mask->height()) 
                    {
                        continue;
                    }
                    auto q = id[nx][ny];
                    if (q == -1) 
                    {
                        A.coeffRef(i, i) += 1;
                        for (int k = 0; k < data_->channels(); k++)
                        {
                            b[k](i) = b[k](i) + data_->get_pixel(nx+static_cast<int>(mouse_position_.x)-static_cast<int>(source_image_->get_position().x),ny+static_cast<int>(mouse_position_.y)-static_cast<int>(source_image_->get_position().y))[k];
                        } 
                    }
                    else 
                    {
                        A.coeffRef(i,i) = A.coeffRef(i,i) + 1;
                        A.coeffRef(i,q) = -1;
                    }
                    for (int k = 0; k < data_->channels(); k++)
                    {
                        b[k](i) = b[k](i) + source_image_->get_data()->get_pixel(x,y)[k] - source_image_->get_data()->get_pixel(nx,ny)[k];
                    }
                    
                }
            }
            A.finalize();
            // Given the Solution
            std::vector<Eigen::VectorXd> X(data_->channels());
            for (int k = 0; k < data_->channels(); k++)
            {
                Eigen::SparseLU<Eigen::SparseMatrix<double>> solver;
                solver.compute(A);
                X[k] = solver.solve(b[k]);
            }
            // Draw the picture
            for (int i = 0; i < mask->width(); ++i)
            {
                for (int j = 0; j < mask->height(); ++j)
                {
                    int tar_x =
                        static_cast<int>(mouse_position_.x) + i -
                        static_cast<int>(source_image_->get_position().x);
                    int tar_y =
                        static_cast<int>(mouse_position_.y) + j -
                        static_cast<int>(source_image_->get_position().y);
                    if (0 <= tar_x && tar_x < image_width_ && 0 <= tar_y &&
                        tar_y < image_height_ && mask->get_pixel(i, j)[0] > 0)
                    {
                        int p = id[i][j];
                        std::vector<uchar> col;
                        for (int ch = 0; ch < data_->channels(); ch++) {
                            col.push_back(static_cast<uchar>(std::clamp(X[ch][p], 0., 255.)));
                        }
                        data_->set_pixel(tar_x, tar_y, col);
                    }

                }
            }
}
}