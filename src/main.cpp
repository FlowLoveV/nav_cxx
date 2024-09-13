#include <iostream>
#include <type_traits>

class MyClass {
public:
    void memberFunc(int) {
        std::cout << "Member function called\n";
    }
};

// 使用模板参数来表示成员函数指针类型
template <typename T, typename ReturnType, typename... Args>
void callMemberFunction(T& obj, ReturnType (T::*func)(Args...), Args... args) {
    (obj.*func)(std::forward<Args>(args)...);  // 调用成员函数
}

int main() {
    MyClass obj;
    callMemberFunction(obj, &MyClass::memberFunc, 42);  // 调用 MyClass::memberFunc
    return 0;
}