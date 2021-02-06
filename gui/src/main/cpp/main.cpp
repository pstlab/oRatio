#include <jni.h>
#include <iostream>

int main(int argc, char const *argv[])
{
    JavaVM *jvm; // Pointer to the JVM (Java Virtual Machine)
    JNIEnv *env; // Pointer to native interface

    JavaVMInitArgs vm_args;                      // Initialization arguments
    JavaVMOption *options = new JavaVMOption[2]; // JVM invocation options
    // where to find java .class
    options[0].optionString = "-Djava.class.path=lib/annotations-13.0.jar;lib/jackson-annotations-2.12.1.jar;lib/jackson-core-2.12.1.jar;lib/jackson-databind-2.12.1.jar;lib/javalin-3.13.3.jar;lib/javax.servlet-api-3.1.0.jar;lib/jetty-client-9.4.35.v20201120.jar;lib/jetty-http-9.4.35.v20201120.jar;lib/jetty-io-9.4.35.v20201120.jar;lib/jetty-security-9.4.35.v20201120.jar;lib/jetty-server-9.4.35.v20201120.jar;lib/jetty-servlet-9.4.35.v20201120.jar;lib/jetty-util-9.4.35.v20201120.jar;lib/jetty-util-ajax-9.4.35.v20201120.jar;lib/jetty-webapp-9.4.35.v20201120.jar;lib/jetty-xml-9.4.35.v20201120.jar;lib/kotlin-stdlib-1.3.71.jar;lib/kotlin-stdlib-common-1.3.71.jar;lib/kotlin-stdlib-jdk7-1.3.71.jar;lib/kotlin-stdlib-jdk8-1.3.71.jar;lib/oratio-1.0.jar;lib/slf4j-api-1.7.30.jar;lib/slf4j-simple-1.7.30.jar;lib/slf4j-api-1.7.30.jar;lib/websocket-api-9.4.35.v20201120.jar;lib/websocket-client-9.4.35.v20201120.jar;lib/websocket-common-9.4.35.v20201120.jar;lib/websocket-server-9.4.35.v20201120.jar;lib/websocket-servlet-9.4.35.v20201120.jar;gui-1.0.jar";
    options[1].optionString = "-Djava.library.path=../../build/api/java/src/main/cpp/solver";
    vm_args.version = JNI_VERSION_10; // minimum Java version
    vm_args.nOptions = 2;             // number of options
    vm_args.options = options;
    vm_args.ignoreUnrecognized = false; // invalid options make the JVM init fail

    jint rc = JNI_CreateJavaVM(&jvm, (void **)&env, &vm_args);
    delete options; // we then no longer need the initialisation options.
    if (rc == JNI_OK)
    {
        jclass app = env->FindClass("it/cnr/istc/pst/oratio/gui/App");
        std::cout << app << std::endl;
    }

    return 0;
}
