#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <QAction>
#include <QFileDialog>
#include <QMessageBox>
#include <QTreeWidget>
#include <QTableWidget>
#include <QComboBox>
#include <QCheckBox>
#include <QMenu>

#include "Tools.h"
#include "PointCloud/renderingWidget.h"

#include "MyCloud.h"

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::MainWindow)
{
    ui->setupUi(this);

	/** 初始化OpenGL渲染窗口 */
	renderWidget = new RenderWidget(this);
	setCentralWidget(renderWidget);
	renderWidget->setMouseTracking(true);

    init();

	this->installEventFilter(this); //对整个程序安装事件过滤器
}

MainWindow::~MainWindow()
{
    delete ui;
}

/************************************************************************/
/*                 初始化界面参数，设置主界面信号槽                        */
/************************************************************************/
void MainWindow::init()
{
	// 设置默认主题
	QFile rfile(":/Resources/qss/Darcula.qss");
	rfile.open(QFile::ReadOnly);
	QTextStream ts(&rfile);
	qApp->setStyleSheet(ts.readAll());

	// 设置字体
	QFont font("Microsoft YaHei");
	font.setPointSize(9.0);
	qApp->setFont(font);

	// 设置dataDock悬浮状态
	this->resize(1600, 900);
	ui->dataDock->resize(300, 620);
	ui->dataDock->setFloating(true);
	ui->dataDock->setAllowedAreas(Qt::NoDockWidgetArea);

	// 设置表头填充满表格
	ui->propertyTable->horizontalHeader()->setSectionResizeMode(1, QHeaderView::Stretch);

	/***** Slots connection of dataTree(QTreeWidget) widget *****/
	// Item in dataTree is left-clicked (connect)
	connect(ui->dataTree, &QTreeWidget::itemClicked, this, &MainWindow::itemSelected);
	// Item in dataTree is right-clicked
	connect(ui->dataTree, SIGNAL(customContextMenuRequested(const QPoint&)), this, SLOT(popMenu(const QPoint&)));
	connect(ui->actionOpen, &QAction::triggered, this, &MainWindow::openFile);
}

bool MainWindow::eventFilter(QObject* obj, QEvent* event)
{
	/** 使dataDock随主界面的移动缩放所移动 */
	
	if (obj == this)
	{
		if (event->type() == QEvent::Resize)
		{
			//QMainWindow::resizeEvent(event);
			int posX = this->geometry().x();
			int posY = this->geometry().y();
			int mainWindowWidth = this->geometry().width();
			int mainWindowHeight = this->geometry().height();

			if (mainWindowWidth < 1360 || mainWindowHeight < 780)
			{
				ui->dataDock->resize(250, 405);
			}

			ui->dataDock->move(posX + mainWindowWidth - ui->dataDock->geometry().width() - 50, posY + 100);

			return true;
		}

		if (event->type() == QEvent::Move && QEvent::MouseButtonPress)
		{
			int posX = this->geometry().x();
			int posY = this->geometry().y();
			int mainWindowWidth = this->geometry().width();
			int mainWindowHeight = this->geometry().height();

			ui->dataDock->move(posX + mainWindowWidth - ui->dataDock->geometry().width() - 50, posY + 100);

			return true;
		}
	}
	/** 如果没有捕获到自定义事件，则将事件返回给程序执行默认操作 */
	return QWidget::eventFilter(obj, event);
}

void MainWindow::debug(const QString& s)
{
	QMessageBox::information(this, tr("Debug"), s);
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

		// 保存pcl格式点云
		mycloud_vec.emplace_back(mycloud_);

		/************************************************************************/
		/*     renderWidget类添加点云、mesh等形状的方法，添加到渲染队列中           */
		/************************************************************************/
		renderWidget->addPointCloud(pcd);

		// 添加到dataTree中显示
		QTreeWidgetItem* cloudName = new QTreeWidgetItem(QStringList()
			<< toQString(fileName));
		cloudName->setIcon(0, QIcon(":/Resources/images/kdtree_done.png"));
		cloudName->setCheckState(0, Qt::CheckState::Checked);
		ui->dataTree->addTopLevelItem(cloudName);
	}
}

void MainWindow::setPropertyTable(const int count)
{
	ui->propertyTable->clearContents();
	ui->propertyTable->setColumnCount(2);
	ui->propertyTable->setRowCount(7);

	/** 选择按什么属性值渲染 */
	QComboBox* renderValueComboBox = new QComboBox(this);
	renderValueComboBox->insertItem(0, QIcon(":/Resources/images/none.png"), "None");
	int valueIndex = 1;
	if (mycloud_vec[count].hasRgb)
	{
		renderValueComboBox->insertItem(valueIndex, QIcon(":/Resources/images/RGBColor.png"), tr("RGB"));
		valueIndex++;
	}
	if (mycloud_vec[count].hasIntensity)
	{
		renderValueComboBox->insertItem(valueIndex, QIcon(":/Resources/images/IntensityColor.png"), tr("Intensity"));
		valueIndex++;
	}
	renderValueComboBox->insertItem(valueIndex, QIcon(":/Resources/images/zValue.png"), tr("Height"));

	/** 设置属性窗口的色带类型选项 */
	QComboBox* colorTypeComboBox = new QComboBox(this);
	colorTypeComboBox->insertItem(0, QIcon(":/Resources/images/colormap_cool.png"), "Cool");
	colorTypeComboBox->insertItem(1, QIcon(":/Resources/images/colormap_gray.png"), "Gray");
	colorTypeComboBox->insertItem(2, QIcon(":/Resources/images/colormap_hot.png"), "Hot");
	colorTypeComboBox->insertItem(3, QIcon(":/Resources/images/colormap_jet.png"), "Jet");
	int iconW = colorTypeComboBox->geometry().width() * 3.0 / 4.0;
	int iconH = colorTypeComboBox->geometry().height();
	colorTypeComboBox->setIconSize(QSize(iconW, iconH));

	/** 设置点云的点尺寸 */
	QComboBox* setSize = new QComboBox(this);
	setSize->addItems({ "1","2","3","4","5" ,"6","7","8","9" });

	/** 设置EDL模式 */
	QCheckBox* edlCheckBox = new QCheckBox(this);
	edlCheckBox->setText(tr("Enable EDL"));
	edlCheckBox->setChecked(renderWidget->getEDLState());

	/** 设置当前渲染点云的属性 */
	QString cloud_id = toQString(mycloud_vec[count].cloudId);	// 获取当前选择点云的名称
	int cloud_size = renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->points_num;
	// 设置当前渲染属性
	int renderValue = renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->getAttributeMode();
	switch (renderValue)
	{
	case 1:
		renderValueComboBox->setCurrentText("RGB");
		colorTypeComboBox->setEnabled(false);
		break;
	case 3:
		renderValueComboBox->setCurrentText("Intensity");
		colorTypeComboBox->setEnabled(true);
		break;
	case 4:
		renderValueComboBox->setCurrentText("Height");
		colorTypeComboBox->setEnabled(true);
		break;
	case 5:
		renderValueComboBox->setCurrentText("None");
		colorTypeComboBox->setEnabled(false);
		break;
	default:
		break;
	}
	// 设置当前色带类型
	int colorType = renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->getColorStripType();
	colorTypeComboBox->setCurrentIndex(colorType);
	// 设置当前点云大小
	float pointsize = renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->getPointSize();
	setSize->setCurrentIndex(pointsize - 1);

	/** 设置布局 */
	ui->propertyTable->setItem(0, 0, new QTableWidgetItem("Clouds"));
	ui->propertyTable->setItem(0, 1, new QTableWidgetItem(cloud_id));

	ui->propertyTable->setItem(1, 0, new QTableWidgetItem("Points"));
	ui->propertyTable->setItem(1, 1, new QTableWidgetItem(QString::number(cloud_size)));

	ui->propertyTable->setItem(2, 0, new QTableWidgetItem("Faces"));
	ui->propertyTable->setItem(2, 1, new QTableWidgetItem("0"));

	ui->propertyTable->setItem(3, 0, new QTableWidgetItem("Render value"));
	ui->propertyTable->setCellWidget(3, 1, renderValueComboBox);

	ui->propertyTable->setItem(4, 0, new QTableWidgetItem("Colors"));
	ui->propertyTable->setCellWidget(4, 1, colorTypeComboBox);

	ui->propertyTable->setItem(5, 0, new QTableWidgetItem("Points size"));
	ui->propertyTable->setCellWidget(5, 1, setSize);

	ui->propertyTable->setItem(6, 0, new QTableWidgetItem("EDL"));
	ui->propertyTable->setCellWidget(6, 1, edlCheckBox);

	/** 属性窗口信号槽 */
	// lambda表达式按引用传递会出现错误，改用值传递
	connect(renderValueComboBox, &QComboBox::currentTextChanged, this, [=](const QString& curText) {
		pAttributeChanged(curText);
		if (curText == "None" || curText == "RGB")
			colorTypeComboBox->setEnabled(false);
		else
			colorTypeComboBox->setEnabled(true);
		}
	, Qt::UniqueConnection);
	// 信号函数有重载的时候使用QOverload<>指定使用的参数版本
	connect(colorTypeComboBox, QOverload<int>::of(&QComboBox::currentIndexChanged), this, &MainWindow::pColorMapChanged, Qt::UniqueConnection);
	connect(setSize, &QComboBox::currentTextChanged, this, &MainWindow::pSizeChanged, Qt::UniqueConnection);
	connect(edlCheckBox, &QCheckBox::stateChanged, this, [&](int state) {
		if (state == Qt::Checked)
			renderWidget->setEDL(true);
		if (state == Qt::Unchecked)
			renderWidget->setEDL(false);
		}, Qt::UniqueConnection);
}


void MainWindow::pAttributeChanged(const QString& text)
{
	QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
	int selected_item_count = ui->dataTree->selectedItems().size();
	if (selected_item_count == 1) {
		int cloud_index = ui->dataTree->indexOfTopLevelItem(itemList[0]);
		QString cloud_id = toQString(mycloud_vec[cloud_index].cloudId);
		if (text == "None")
		{
			renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setAttributeMode(NO_COLOR);
		}
		else if (text == "RGB")
		{
			renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setAttributeMode(FROM_RBG);
		}
		else if (text == "Intensity")
		{
			renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setAttributeMode(FROM_INTENSITY);
		}
		else if (text == "Height")
		{
			renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setAttributeMode(FROM_HEIGHT);
		}
		//ui.statusBar->showMessage(tr("Change point cloud render attribute success"), 3000);
	}
	else
		debug(tr("Please choose one cloud"));
}

void MainWindow::pColorMapChanged(int color_type)
{
	QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
	int selected_item_count = ui->dataTree->selectedItems().size();
	if (selected_item_count == 1) {
		int cloud_index = ui->dataTree->indexOfTopLevelItem(itemList[0]);
		QString cloud_id = toQString(mycloud_vec[cloud_index].cloudId);
		switch (color_type)
		{
		case 0:
			renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setColorStripType(COLOR_COOL);
			break;
		case 1:
			renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setColorStripType(COLOR_GRAY);
			break;
		case 2:
			renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setColorStripType(COLOR_HOT);
			break;
		case 3:
			renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setColorStripType(COLOR_JET);
			break;
		default:
			break;
		}
	}
	else
		debug(tr("Please choose one cloud"));
}

void MainWindow::pSizeChanged(const QString& text)
{
	unsigned int p = text.toFloat();
	QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
	int selected_item_count = ui->dataTree->selectedItems().size();
	if (selected_item_count == 0) {
		return;
	}
	else {
		for (int i = 0; i != selected_item_count; i++) {
			int cloud_index = ui->dataTree->indexOfTopLevelItem(itemList[i]);
			QString cloud_id = toQString(mycloud_vec[cloud_index].cloudId);
			renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setPointSize(p);
		}
		// 输出窗口
		//ui.statusBar->showMessage(tr("Change selected point cloud size"), 3000);
	}
}

void MainWindow::itemSelected(QTreeWidgetItem* item, int count)
{
	//获取item的行号
	count = ui->dataTree->indexOfTopLevelItem(item);

	//绘制属性窗口
	setPropertyTable(count);

	QString cloud_id = toQString(mycloud_vec[count].cloudId);	// 获取当前选择点云的名称
	bool cloudVisible = renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->getVisible();
	//点击data tree点云前checkbox显示或者隐藏点云
	if (item->checkState(0) == Qt::CheckState::Checked && cloudVisible == false)
	{
		showItem(count);
	}
	else if (item->checkState(0) == Qt::CheckState::Unchecked && cloudVisible == true)
	{
		hideItem(count);
	}
}

void MainWindow::popMenu(const QPoint&)
{
	QTreeWidgetItem* curItem = ui->dataTree->currentItem(); //获取当前被点击的节点
	if (curItem == NULL) return;           //这种情况是右键的位置不在treeItem的范围内，即在空白位置右击
	QString name = curItem->text(0);
	int id = ui->dataTree->indexOfTopLevelItem(curItem);
	MyCloud myCloud_ = mycloud_vec[id];

	QAction* deleteItemAction = new QAction(tr("Delete"), this);
	QAction* kdTreeAction = new QAction(tr("Bulid kdTree"), this);

	connect(deleteItemAction, &QAction::triggered, this, &MainWindow::deleteItem);
	connect(kdTreeAction, &QAction::triggered, this, [&]() {
		if (mycloud_vec[id].KdTree_mycloud)
			return;
		curItem->setIcon(0, QIcon(":/Resources/images/kdtree_done.png"));
		mycloud_vec[id].KdTree_mycloud.reset(new pcl::search::KdTree<PointT>);
		mycloud_vec[id].KdTree_mycloud->setInputCloud(mycloud_vec[id].cloud);
		});

	QPoint pos;
	QMenu* menu = new QMenu(ui->dataTree);
	menu->addAction(deleteItemAction);
	menu->addAction(kdTreeAction);

	menu->exec(QCursor::pos()); //在当前鼠标位置显示
}

void MainWindow::hideItem(const int index)
{
	//QList<QTreeWidgetItem*> itemList;
	//for (int i = 0; i < ui->dataTree->topLevelItemCount(); i++)
	//{
	//	itemList.push_back(ui->dataTree->topLevelItem(i));
	//}
	////QTreeWidgetItem* curItem = ui.dataTree->currentItem();
	QTreeWidgetItem* curItem = ui->dataTree->topLevelItem(index);
	QString cloud_id = toQString(mycloud_vec[index].cloudId);
	//int id = ui.dataTree->indexOfTopLevelItem(curItem);
	if (curItem->checkState(0) == Qt::Unchecked)
	{
		renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setVisible(false);
		
		QColor item_color(50, 50, 50, 255);
		curItem->setTextColor(0, item_color);
	}
	else
	{
		return;
	}
	// 输出窗口
	//ui.statusBar->showMessage(tr("Hide selected point cloud"), 3000);

}

void MainWindow::showItem(const int index)
{
	////QList<QTreeWidgetItem*> itemList = ui.dataTree->selectedItems();
	//QList<QTreeWidgetItem*> itemList;
	//for (int i = 0; i < ui.dataTree->topLevelItemCount(); i++)
	//{
	//	itemList.push_back(ui.dataTree->topLevelItem(i));
	//}

	QTreeWidgetItem* curItem = ui->dataTree->topLevelItem(index);
	QString cloud_id = toQString(mycloud_vec[index].cloudId);
	//int id = ui.dataTree->indexOfTopLevelItem(curItem);
	if (curItem->checkState(0) == Qt::Checked)
	{// 将cloud_id所对应的点云设置成透明
		//mycloud_vec[index].show();
		renderWidget->getProgressiveRenderMap().at(cloud_id)->getCurrentPcd()->setVisible(true);

		QColor item_color(241, 241, 241, 255);
		curItem->setTextColor(0, item_color);
	}
	else
	{
		return;
	}

	// 输出窗口
	//ui.statusBar->showMessage(tr("Show selected point cloud"), 3000);
}

void MainWindow::deleteItem()
{
	QList<QTreeWidgetItem*> itemList = ui->dataTree->selectedItems();
	// ui.dataTree->selectedItems().size() 随着迭代次数而改变，因此循环条件要设置为固定大小的 selected_item_count
	int selected_item_count = ui->dataTree->selectedItems().size();
	for (int i = 0; i != selected_item_count; i++) {
		QTreeWidgetItem* curItem = itemList[i];
		QString name = curItem->text(0);
		int id = ui->dataTree->indexOfTopLevelItem(curItem);
		//QMessageBox::information(this, "information", "curItem: " + name + " " + QString::number(id));
		auto itCloud = mycloud_vec.begin() + id;
		// 从viewer中删除点云
		renderWidget->removePointCloud(toQString((*itCloud).cloudId));
		// 删除vector容器与数据列表的点云对象
		itCloud = mycloud_vec.erase(itCloud);
		ui->dataTree->takeTopLevelItem(id);
	}
	mycloud_vec.shrink_to_fit();
	ui->propertyTable->clearContents();  //清空属性窗口propertyTable

	// 输出窗口
	//ui.statusBar->showMessage(tr("Delete selected point cloud"), 3000);
}

