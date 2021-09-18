//
// Created by quantum on 9/15/21.
//

#include "torch/torch.h"

int main()
{
   torch::Tensor x = torch::randn({3,3});
   std::cout << x << std::endl;
}