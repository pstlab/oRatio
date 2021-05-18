#include <jni.h>
#include <thread>
#include <condition_variable>
#include <mutex>
#include "solver.h"

using namespace ratio;

int main(int argc, char const *argv[])
{
    if (argc < 3)
    {
        std::cerr << "usage: oRatio <input-file> [<input-file> ...] <output-file>\n";
        return -1;
    }

    // the problem files..
    std::vector<std::string> prob_names;
    for (int i = 1; i < argc - 1; i++)
        prob_names.push_back(argv[i]);

    // the solution file..
    std::string sol_name = argv[argc - 1];

    JavaVM *jvm; // Pointer to the JVM (Java Virtual Machine)
    JNIEnv *env; // Pointer to native interface

    JavaVMInitArgs vm_args;                      // Initialization arguments
    JavaVMOption *options = new JavaVMOption[2]; // JVM invocation options
    // where to find java .class
    options[0].optionString = (char *)"-Djava.class.path=lib/annotations-13.0.jar;lib/jackson-annotations-2.12.1.jar;lib/jackson-core-2.12.1.jar;lib/jackson-databind-2.12.1.jar;lib/javalin-3.13.3.jar;lib/javax.servlet-api-3.1.0.jar;lib/jetty-client-9.4.35.v20201120.jar;lib/jetty-http-9.4.35.v20201120.jar;lib/jetty-io-9.4.35.v20201120.jar;lib/jetty-security-9.4.35.v20201120.jar;lib/jetty-server-9.4.35.v20201120.jar;lib/jetty-servlet-9.4.35.v20201120.jar;lib/jetty-util-9.4.35.v20201120.jar;lib/jetty-util-ajax-9.4.35.v20201120.jar;lib/jetty-webapp-9.4.35.v20201120.jar;lib/jetty-xml-9.4.35.v20201120.jar;lib/kotlin-stdlib-1.3.71.jar;lib/kotlin-stdlib-common-1.3.71.jar;lib/kotlin-stdlib-jdk7-1.3.71.jar;lib/kotlin-stdlib-jdk8-1.3.71.jar;lib/oratio-1.0.jar;lib/slf4j-api-1.7.30.jar;lib/slf4j-simple-1.7.30.jar;lib/slf4j-api-1.7.30.jar;lib/websocket-api-9.4.35.v20201120.jar;lib/websocket-client-9.4.35.v20201120.jar;lib/websocket-common-9.4.35.v20201120.jar;lib/websocket-server-9.4.35.v20201120.jar;lib/websocket-servlet-9.4.35.v20201120.jar;gui-1.0.jar";
    options[1].optionString = (char *)"-Djava.library.path=../../build/bin";
    vm_args.version = JNI_VERSION_1_8; // minimum Java version
    vm_args.nOptions = 2;              // number of options
    vm_args.options = options;
    vm_args.ignoreUnrecognized = false; // invalid options make the JVM init fail

    jint rc = JNI_CreateJavaVM(&jvm, (void **)&env, &vm_args);
    delete options; // we then no longer need the initialisation options.
    if (rc == JNI_OK)
    {
        jclass app_class = env->FindClass("it/cnr/istc/pst/oratio/gui/App");
        jmethodID main_mthd = env->GetStaticMethodID(app_class, "start_server", "()V");
        env->CallStaticVoidMethod(app_class, main_mthd);
        std::this_thread::sleep_for(std::chrono::seconds(5));

        jfieldID slv_field = env->GetStaticFieldID(app_class, "SOLVER", "Lit/cnr/istc/pst/oratio/Solver;");
        jobject j_solver = env->GetStaticObjectField(app_class, slv_field);
        solver *s = reinterpret_cast<solver *>(env->GetLongField(j_solver, env->GetFieldID(env->GetObjectClass(j_solver), "native_handle", "J")));

        std::cout << "parsing input files..\n";
        s->read(prob_names);

        std::cout << "solving the problem..\n";
        s->solve();
        std::cout << "hurray!! we have found a solution..\n";

        std::condition_variable cv;
        std::mutex m;
        std::unique_lock<std::mutex> lock(m);
        cv.wait(lock, [] { return false; });
    }

    return 0;
}
