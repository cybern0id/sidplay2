#include "sidedit.h"
#include <QtGui/QApplication>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    SidEdit w;
    w.show();
    return a.exec();
}
