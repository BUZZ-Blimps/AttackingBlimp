#include <functional>
#include <iostream>
#include <string>
#include <typeinfo>

using namespace std;
using namespace std::placeholders;

class ClassB{
    public:
        function<void(string)> callback;

        void update(int i){
            string message = "hi! i=" + to_string(i);
            if(callback) callback(message);
        }
};

class ClassA{
    private:
        ClassB b;

        function<void(string)> _callback;

    public:
        void init(){
            b.callback = bind(&ClassA::callback, this, _1);
        }

        void setCallback(function<void(string)> newCallback){
            _callback = newCallback;
        }

        void update1(int i){
            b.update(i);
        }

        void update2(string msg){
            if(_callback) _callback(msg);
        }

        void callback(string message){
            cout << "Callback: " << message << endl;
        }
};

void printTest(string message){
    cout << "PrintTest: " << message << endl;
}


int main(){
    ClassA a;
    a.update1(1);
    a.init();
    a.update1(2);

    a.update2("3");
    a.setCallback(printTest);
    a.update2("4");

    return 0;
}