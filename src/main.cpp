#include "src/PCLViewer.h"
#include "src/mainwindow.h"

int main(int argc, char *argv[])
{
    PCLViewerapp app(argc, argv);

    MainWindow main_window;
    main_window.show();

    return app.exec();
}

