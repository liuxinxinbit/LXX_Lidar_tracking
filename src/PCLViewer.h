#ifndef PCLVIEWERAPP_H
#define PCLVIEWERAPP_H

#include<QApplication>

class PCLViewerapp : public QApplication
{
    Q_OBJECT
public:
    explicit PCLViewerapp(int &argc, char *argv[]);
};
#endif
