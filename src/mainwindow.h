#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCoreApplication> 

#include "FileIO.h"

class RenderWidget;

QT_BEGIN_NAMESPACE
namespace Ui { class MainWindow; }
QT_END_NAMESPACE

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private:
    Ui::MainWindow *ui;

	RenderWidget* renderWidget = nullptr;

    FileIO fileIO;
	QString dirPath = QCoreApplication::applicationDirPath();//exe文件所在路径

protected:

    void init();
    void openFile();
    void doOpen(QStringList fileNameList);

    void debug(const string& s);

};
#endif // MAINWINDOW_H
