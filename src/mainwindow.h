#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCoreApplication> 

#include "FileIO.h"

class RenderWidget;
class QTreeWidgetItem;

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

	std::vector<MyCloud> mycloud_vec;
    FileIO fileIO;
	QString dirPath = QCoreApplication::applicationDirPath();//exe文件所在路径

	// 程序初始化设置
	void init();
	// 重载事件响应函数，以自定义事件的操作
	virtual bool eventFilter(QObject* obj, QEvent* event) override;
protected:
	void debug(const QString& s);

    void openFile();
    void doOpen(QStringList fileNameList);

	void setPropertyTable(const int count);
   

private slots:
	/***** Slots of propertyTable(QTreeWidget) widget *****/
	void pAttributeChanged(const QString& text);
	void pColorMapChanged(int color_type);
	void pSizeChanged(const QString& text);


	/***** Slots of dataTree(QTreeWidget) widget *****/
	// Item in dataTree is left-clicked
	void itemSelected(QTreeWidgetItem*, int);
	// Item in dataTree is right-clicked
	void popMenu(const QPoint&);
	// method
	void hideItem(const int index);
	void showItem(const int index);
	void deleteItem();
};
#endif // MAINWINDOW_H
