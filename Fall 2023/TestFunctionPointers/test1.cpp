#include <functional>
#include <iostream>
#include <string>
#include <typeinfo>

using namespace std;
using namespace std::placeholders;

class ClassA{    
    public:
        void printI(int i){
            cout << "ClassA: i=" << i << endl;
        }
};

class ClassB{
    public:
        void printO(int o){
            cout << "ClassB: o=" << o << endl;
        }
};

int main(){
    ClassA a;
    ClassB b;
    
    function<void(int)> f1 = bind(&ClassA::printI, &a, _1);
    f1(10);

    f1 = bind(&ClassB::printO, &b, _1);
    f1(20);

    function<void(int)> f2;
    if(f2) f2(30);
    f2 = f1;
    if(f2) f2(40);

    return 0;
}