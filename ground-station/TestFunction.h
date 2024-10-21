#ifndef TESTFUNCTION_H
#define TESTFUNCTION_H

#include <QObject>
#include <QDebug>
#include <qqmlintegration.h>

class TestFunction : public QObject {
    Q_OBJECT
    QML_ELEMENT
public:
    explicit TestFunction(QObject *parent = nullptr) : QObject(parent) {}

public slots:
    void myFunction() {
        qDebug() << "C++ function called from QML";
    }
};

#endif // TESTFUNCTION_H