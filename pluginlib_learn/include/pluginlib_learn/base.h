//
// Created by yuchen on 2022/2/25.
//

#ifndef BASE_H_
#define BASE_H_

namespace base_class
{
    class regular
    {
    public:
        //pluginlib要求构造函数不能带有参数，所以定义initialize来完成需要初始化的工作
        virtual void initialize(double size_length)=0;

        virtual double square()=0;

         virtual ~regular(){};                               //please remenber ~regular(){};  but not ~regular();,it will flase

    protected:
        regular(){};
    };
}

#endif
