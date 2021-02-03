#include <jni.h>
#include <iostream>

int main(int argc, char const *argv[])
{
    std::cout << "Hi!" << std::endl;

    JavaVM *jvm; // Pointer to the JVM (Java Virtual Machine)
    JNIEnv *env; // Pointer to native interface

    JavaVMInitArgs vm_args;                          // Initialization arguments
    JavaVMOption *options = new JavaVMOption[1];     // JVM invocation options
    options[0].optionString = "-Djava.class.path=."; // where to find java .class
    vm_args.version = JNI_VERSION_1_8;               // minimum Java version
    vm_args.nOptions = 1;                            // number of options
    vm_args.options = options;
    vm_args.ignoreUnrecognized = false; // invalid options make the JVM init fail

    jint rc = JNI_CreateJavaVM(&jvm, (void **)&env, &vm_args);
    delete options; // we then no longer need the initialisation options.
    if (rc == JNI_OK)
    {
        jclass app = env->FindClass("it.cnr.istc.pst.oratio.gui.App"); // try to find the class
    }

    return 0;
}
