#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QAction>
#include <QFileDialog>
#include <QMessageBox>

#include "Tools.h"
#include "PointCloud/renderingWidget.h"

#include "MyCloud.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

    connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::openFile);

    init();
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::init()
{
    renderWidget = new RenderWidget(this);
	setCentralWidget(renderWidget);

	//this->setMouseTracking(true);
	//renderWidget->installEventFilter(this);
	renderWidget->setMouseTracking(true);
}

void MainWindow::openFile()
{
	QStringList filePathList = QFileDialog::getOpenFileNames(
		this,
		tr("Open point cloud file"),
		"",
		toQString(fileIO.getInputFormatsStr())
	);
	if (filePathList.isEmpty()) return;

	doOpen(filePathList);
}

void MainWindow::doOpen(QStringList filePathList)
{
	// Open point cloud file one by one
	for (int i = 0; i != filePathList.size(); i++) {
		// 读取文件
		QFileInfo fileInfo(filePathList[i]);
		std::string filePath = fileInfo.filePath().toLocal8Bit();
		std::string fileName = fileInfo.fileName().toLocal8Bit();
		QString cloud_id = fileInfo.baseName();
		auto allRenderActor = renderWidget->getProgressiveRenderMap();
		std::map<QString, ProgressiveRender*>::iterator cloud_it = allRenderActor.find(cloud_id);
		if (cloud_it != allRenderActor.end()) {
			qDebug() << tr("[openFile] A cloud with name ") + cloud_id + tr(" already exists!") << endl;
			continue;
		}
		// OpenGL渲染格式点云
		std::shared_ptr<PointCloud> pcd(new PointCloud());

		// pcl点云格式，计算时使用
		MyCloud mycloud_ = (fileIO.load(fileInfo, pcd));

		if (!mycloud_.isValid) {
			// TODO: deal with the error, print error info in console?
			debug("invalid cloud.");
			continue;
		}

		//mycloud_vec.emplace_back(mycloud_);

		/************************************************************************/
		/*          renderWidget类添加点云、mesh等形状的方法                      */
		/************************************************************************/
		renderWidget->addPointCloud(pcd);

		//// update tree widget
		//QTreeWidgetItem* cloudName = new QTreeWidgetItem(QStringList()
		//	<< toQString(fileName));
		//cloudName->setIcon(0, QIcon(":/Resources/images/kdtree_done.png"));
		//cloudName->setCheckState(0, Qt::CheckState::Checked);
		//ui.dataTree->addTopLevelItem(cloudName);
		////total_points += mycloud_.cloud->points.size();

		//total_points += pcd->points_num;
		/*mycloud_.cloud.reset();
		mycloud_.mesh.reset();
		mycloud_.viewer.reset();
		mycloud_.KdTree_mycloud.reset();*/


	}
}

void MainWindow::debug(const string& s)
{
	QMessageBox::information(this, tr("Debug"), toQString(s));
}

