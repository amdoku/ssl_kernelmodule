#include <string>

class FpgaRegion
{
public:
    FpgaRegion(std::string name, void(&reconfig)(), void(&release)()) {}
    void Acquire() {}
    void Release() {}
};
