#include "MyTest.h"
#include <Person.pb.h>
// #include "Address.pb.h"

void MyTest::test()
{
    //序列化
    Person p;
    p.set_id(10);
    p.set_age(32);
    p.set_sex("man");

    p.add_name();
    p.set_name(0, "路飞");
    p.add_name("艾斯");
    p.add_name("萨博");
    p.mutable_addr()->set_addr("北京市长安街天安门");
    p.mutable_addr()->set_num(1001);

    p.set_colcor(Blue);

    //序列化对象 p,最终得到一个字符串
    std::string output;
    p.SerializeToString(&output);

    //反序列化操作
    Person pp;
    pp.ParseFromString(output);
    std::cout << pp.id() << "," << pp.sex() << ","  << "," << pp.age() << std::endl;
    std::cout << pp.addr().addr() << "," << pp.addr().num() << std::endl;
    int size = pp.name_size();
    for(int i=0; i<size; ++i)
    {
        std::cout << pp.name(i) << std::endl;
    }

    std::cout << pp.colcor() << std::endl;
}
