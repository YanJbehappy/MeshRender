#include "FileIO.h"
#include "Tools.h"
#include <QString>
//#include <algorithm>

#include "lasreader.hpp"
#include "laswriter.hpp"

void FileIO::toPointCloudGL(const MyCloud& mycloud)
{
	int points_num = mycloud.cloud->points.size();

	pcd_->name = toQString(mycloud.cloudId);
	pcd_->position.resize(3, points_num); // 位置矩阵
	pcd_->colors.resize(4, points_num); // 颜色矩阵
	pcd_->labels.resize(1, points_num); // 分类信息
	pcd_->intensity.resize(1, points_num); // 强度信息
	pcd_->offset = Eigen::Vector3f(0.0, 0.0, 0.0); // 头文件中包含的偏移

	bool is8bit = false;
	// 某些格式下含有颜色信息
	if (mycloud.hasRgb) {
		if (mycloud.cloud->points[0].a <= 255)
			is8bit = true;
	}
	// Read Point
#pragma omp parallel for
	for (int index = 0; index < points_num; index++) {
		// 读取坐标
		Eigen::Vector3f point3d;
		point3d.x() = mycloud.cloud->points[index].x;
		point3d.y() = mycloud.cloud->points[index].y;
		point3d.z() = mycloud.cloud->points[index].z;
		pcd_->position.col(index) = point3d;

		// 读取颜色
		Eigen::Vector4i_8 color;
		if (mycloud.hasRgb) {
			color.x() = is8bit ? mycloud.cloud->points[index].r : mycloud.cloud->points[index].r / 257;
			color.y() = is8bit ? mycloud.cloud->points[index].g : mycloud.cloud->points[index].g / 257;
			color.z() = is8bit ? mycloud.cloud->points[index].b : mycloud.cloud->points[index].b / 257;
			color.w() = is8bit ? mycloud.cloud->points[index].a : mycloud.cloud->points[index].a / 257;
		}
		// 不含颜色信息则赋白模
		else {
			color.x() = 255;
			color.y() = 255;
			color.z() = 255;
			color.w() = 0;
		}
		pcd_->colors.col(index) = color;

		pcd_->labels(0, index) = (int)0;
		pcd_->intensity(0, index) = (int)mycloud.intensity[index];
	}

	Eigen::Vector3f minp = pcd_->position.rowwise().minCoeff(); // 取出每一列中最小值 得到最小点
	Eigen::Vector3f maxp = pcd_->position.rowwise().maxCoeff(); // 取出每一列中最大值 得到最大点

	pcd_->boundingBox.extend(minp);
	pcd_->boundingBox.extend(maxp);

	pcd_->points_num = points_num;

	// Max Min Idensity
	pcd_->setmaxIdensity(pcd_->intensity.rowwise().maxCoeff()(0, 0));
	pcd_->setminIdensity(pcd_->intensity.rowwise().minCoeff()(0, 0));

	// 分配 indexTable
	pcd_->indexTable = new uint32_t[points_num];
}

void FileIO::readCloudHeader(const pcl::PCLPointCloud2& cloud2, MyCloud& mycloud)
{
	std::vector<std::string> fieldsName;
	for (int i = 0; i < cloud2.fields.size(); i++)
	{
		//防止头文件中的大小写差异，讲所有属性转换为大写
		string member = cloud2.fields[i].name;
		std::transform(member.begin(), member.end(), member.begin(), ::toupper);
		fieldsName.push_back(member);
	}

	std::string RGB = "RGB";
	std::string Intensity = "INTENSITY";

	std::vector<std::string>::iterator it_RGB = std::find(fieldsName.begin(), fieldsName.end(), RGB);
	std::vector<std::string>::iterator it_Intensity = std::find(fieldsName.begin(), fieldsName.end(), Intensity);
	if (it_RGB == fieldsName.end())
		mycloud.hasRgb = false;
	else
		mycloud.hasRgb = true;

	if (it_Intensity == fieldsName.end())
		mycloud.hasIntensity = false;
	else
		mycloud.hasIntensity = true;
}

MyCloud FileIO::loadPLY(const QFileInfo& fileInfo) {
	MyCloud myCloud;
	string filePath = fileInfo.filePath().toLocal8Bit();
	int status = -1;
	//status = pcl::io::loadPLYFile(filePath, *(myCloud.cloud));
	//读取ply文件的点云信息
	pcl::PCLPointCloud2 cloud2;
	status = pcl::io::loadPLYFile(filePath, cloud2);
	pcl::fromPCLPointCloud2(cloud2, *(myCloud.cloud));
	pcl::io::loadPolygonFilePLY(filePath, *(myCloud.mesh));

	readCloudHeader(cloud2, myCloud);

	myCloud.intensity.resize(myCloud.cloud->points.size(), 0.0);

	bool hasCloud = (status == 0);
	bool hasMesh = (myCloud.mesh->polygons.size() != 0);
	myCloud.init(fileInfo, hasCloud, hasMesh);

	// 创建OpenGL渲染格式的点云
	toPointCloudGL(myCloud);
	return myCloud;
}

MyCloud FileIO::loadPCD(const QFileInfo& fileInfo) {
	MyCloud myCloud;
	//string filePath = fromQString(fileInfo.filePath());
	string filePath = fileInfo.filePath().toLocal8Bit();
	int status = -1;
	pcl::PCLPointCloud2 cloud2;
	status = pcl::io::loadPCDFile(filePath, cloud2);
	pcl::fromPCLPointCloud2(cloud2, *(myCloud.cloud));

	readCloudHeader(cloud2, myCloud);
	if (myCloud.hasIntensity)
	{
		pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_(new pcl::PointCloud<pcl::PointXYZI>);
		pcl::io::loadPCDFile(filePath, *cloud_);

		myCloud.intensity.resize(cloud_->points.size());
#pragma omp parallel for
		for (int i = 0; i < cloud_->points.size(); i++)
			myCloud.intensity[i] = cloud_->points[i].intensity;
	}
	else
	{
		myCloud.intensity.resize(myCloud.cloud->points.size(), 0.0);
	}

	bool hasCloud = (status == 0);
	// There is no polygon mesh loader for pcd file in PCL
	bool hasMesh = false;
	myCloud.init(fileInfo, hasCloud, hasMesh);

	toPointCloudGL(myCloud);

	return myCloud;
}

MyCloud FileIO::loadLAS(const QFileInfo& fileInfo)
{
	MyCloud myCloud;

	string filePath = fileInfo.filePath().toLocal8Bit();
	LASreadOpener lasreadopener;
	lasreadopener.set_file_name(filePath.c_str());
	LASreader* lasreader = lasreadopener.open();

	LASheader lasheader = lasreader->header;
	int points_num = lasheader.number_of_point_records ? lasheader.number_of_point_records : lasheader.extended_number_of_point_records;

	pcd_->name = fileInfo.baseName();
	pcd_->position.resize(3, points_num); // 位置矩阵
	pcd_->colors.resize(4, points_num); // 颜色矩阵
	pcd_->labels.resize(1, points_num); // 分类信息
	pcd_->intensity.resize(1, points_num); // 强度信息
	pcd_->offset = Eigen::Vector3f(0.0, 0.0, 0.0); // 头文件中包含的偏移
	// 某些格式下含有颜色信息
	int format = lasheader.point_data_format;
	if (format == 2 || format == 3 || format == 5)
		myCloud.hasRgb = true;

	PointT TempPoint;
	uint16_t r1, g1, b1;
	int index = 0;
	while (lasreader->read_point()) {
		// pcl格式点云
		{
			TempPoint.x = lasreader->point.get_x();
			TempPoint.y = lasreader->point.get_y();
			TempPoint.z = lasreader->point.get_z();
			r1 = lasreader->point.get_R();
			g1 = lasreader->point.get_G();
			b1 = lasreader->point.get_B();
			TempPoint.r = ceil(((float)r1 / 65536) * (float)256);
			TempPoint.g = ceil(((float)g1 / 65536) * (float)256);
			TempPoint.b = ceil(((float)b1 / 65536) * (float)256);
			myCloud.cloud->points.push_back(TempPoint);
			myCloud.intensity.push_back(lasreader->point.get_intensity());
		}

		// OpenGL渲染需要格式点云
		{
			Eigen::Vector3f point3d;
			point3d.x() = TempPoint.x;
			point3d.y() = TempPoint.y;
			point3d.z() = TempPoint.z;

			// 读取颜色
			Eigen::Vector4i_8 color;
			if (myCloud.hasRgb) {

				color.x() = TempPoint.r;
				color.y() = TempPoint.g;
				color.z() = TempPoint.b;
				color.w() = 255;
			}
			// 不含颜色信息则赋白模
			else {
				color.x() = 255;
				color.y() = 255;
				color.z() = 255;
				color.w() = 0;
			}

			pcd_->position.col(index) = point3d;
			pcd_->colors.col(index) = color;
			pcd_->labels(0, index) = (int)lasreader->point.get_classification();
			pcd_->intensity(0, index) = (int)lasreader->point.get_intensity();

			index++;
		}
	}

	Eigen::Vector3f minp = pcd_->position.rowwise().minCoeff(); // 取出每一列中最小值 得到最小点
	Eigen::Vector3f maxp = pcd_->position.rowwise().maxCoeff(); // 取出每一列中最大值 得到最大点

	pcd_->boundingBox.extend(minp);
	pcd_->boundingBox.extend(maxp);

	pcd_->points_num = points_num;

	// Max Min Idensity
	pcd_->setmaxIdensity(pcd_->intensity.rowwise().maxCoeff()(0, 0));
	pcd_->setminIdensity(pcd_->intensity.rowwise().minCoeff()(0, 0));

	if (pcd_->getmaxIdensity() != pcd_->getminIdensity())
	{
		myCloud.hasIntensity = true;
	}
	// 分配 indexTable
	pcd_->indexTable = new uint32_t[points_num];

	bool hasCloud = (myCloud.cloud->size() != 0);
	// There is no polygon mesh loader for las file in PCL
	bool hasMesh = false;
	myCloud.init(fileInfo, hasCloud, hasMesh);

	return myCloud;
}

MyCloud FileIO::loadXYZ(const QFileInfo& fileInfo) {
	MyCloud myCloud;
	PointT TempPoint;
	string filePath = fileInfo.filePath().toLocal8Bit();
	std::ifstream ifs(filePath); // 
	string str;
	bool status = true;
	while (getline(ifs, str))
	{
		istringstream input(str);
		vector<string> tmp;
		string member;
		while (input >> member)
			tmp.push_back(member);

		int col = tmp.size();
		switch (col)
		{
		case 3: //
			TempPoint.x = stod(tmp[0]);
			TempPoint.y = stod(tmp[1]);
			TempPoint.z = stod(tmp[2]);
			myCloud.intensity.push_back(0.0);
			break;
		case 4: //
			TempPoint.x = stod(tmp[0]);
			TempPoint.y = stod(tmp[1]);
			TempPoint.z = stod(tmp[2]);
			myCloud.intensity.push_back(stof(tmp[3]));
			myCloud.hasIntensity = true;
			break;
		case 6: //
			TempPoint.x = stod(tmp[0]);
			TempPoint.y = stod(tmp[1]);
			TempPoint.z = stod(tmp[2]);
			TempPoint.r = stoi(tmp[3]);
			TempPoint.g = stoi(tmp[4]);
			TempPoint.b = stoi(tmp[5]);
			myCloud.intensity.push_back(0.0);
			break;
		case 7:
			TempPoint.x = stod(tmp[0]);
			TempPoint.y = stod(tmp[1]);
			TempPoint.z = stod(tmp[2]);
			TempPoint.r = stoi(tmp[3]);
			TempPoint.g = stoi(tmp[4]);
			TempPoint.b = stoi(tmp[5]);
			myCloud.intensity.push_back(stof(tmp[6]));
			myCloud.hasIntensity = true;
			break;
		default:
			status = false;
			break;
		}
		if (!status) {
			cerr << "Unsupport file format" << endl;
			break;
		}

		myCloud.cloud->points.push_back(TempPoint);
	}

	bool hasCloud = status;
	bool hasMesh = false;
	myCloud.init(fileInfo, hasCloud, hasMesh);

	// 创建OpenGL渲染格式的点云
	toPointCloudGL(myCloud);

	return myCloud;
}

MyCloud FileIO::loadOBJ(const QFileInfo& fileInfo) {
	MyCloud myCloud;
	string filePath = fileInfo.filePath().toLocal8Bit();
	int status = -1;
	pcl::PCLPointCloud2 cloud2;
	status = pcl::io::loadOBJFile(filePath, cloud2);
	pcl::fromPCLPointCloud2(cloud2, *(myCloud.cloud));
	pcl::io::loadPolygonFileOBJ(filePath, *(myCloud.mesh));

	readCloudHeader(cloud2, myCloud);

	myCloud.intensity.resize(myCloud.cloud->points.size(), 0.0);

	bool hasCloud = (status == 0);
	bool hasMesh = (myCloud.mesh->polygons.size() != 0);
	myCloud.init(fileInfo, hasCloud, hasMesh);

	toPointCloudGL(myCloud);
	return myCloud;
}

MyCloud FileIO::loadSTL(const QFileInfo& fileInfo) {
	MyCloud myCloud;
	string filePath = fileInfo.filePath().toLocal8Bit();
	pcl::io::loadPolygonFileSTL(filePath, *(myCloud.mesh));

	bool hasCloud = false;
	bool hasMesh = (myCloud.mesh->polygons.size() != 0);
	myCloud.init(fileInfo, hasCloud, hasMesh);

	return myCloud;
}

MyCloud FileIO::loadVTK(const QFileInfo& fileInfo) {
	MyCloud myCloud;
	string filePath = fileInfo.filePath().toLocal8Bit();
	pcl::io::loadPolygonFileVTK(filePath, *(myCloud.mesh));

	bool hasCloud = false;
	bool hasMesh = (myCloud.mesh->polygons.size() != 0);
	myCloud.init(fileInfo, hasCloud, hasMesh);

	return myCloud;
}

MyCloud FileIO::load(const QFileInfo& fileInfo, std::shared_ptr<PointCloud>& pcd) {
	pcd_.reset();
	pcd_ = pcd;
	string suffix = fromQString(fileInfo.suffix().toLower());
	if (suffix == "ply") {
		return loadPLY(fileInfo);
	}
	else if (suffix == "pcd") {
		return loadPCD(fileInfo);
	}
	else if (suffix == "las") {
		return loadLAS(fileInfo);
	}
	else if (suffix == "xyz" || suffix == "txt") {
		return loadXYZ(fileInfo);
	}
	else if (suffix == "obj") {
		return loadOBJ(fileInfo);
	}
	else if (suffix == "stl") {
		return loadSTL(fileInfo);
	}
	else if (suffix == "vtk") {
		return loadVTK(fileInfo);
	}
	else {
		return MyCloud::getInvalidMyCloud();
	}
}

bool FileIO::savePLY(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat) {
	if (!myCloud.isValid) return false;
	string filePath = fileInfo.filePath().toLocal8Bit();
	if (myCloud.hasMesh) {
		return pcl::io::savePolygonFilePLY(filePath, *myCloud.mesh, isBinaryFormat);
	}
	else {
		int status = pcl::io::savePLYFile(filePath, *myCloud.cloud, isBinaryFormat);
		return status == 0;
	}
}

bool FileIO::saveOBJ(const MyCloud& myCloud, const QFileInfo& fileInfo) {
	if (!myCloud.hasMesh) return false;
	string filePath = fileInfo.filePath().toLocal8Bit();
	// pcl::io::saveOBJFile() does not support binary format
	int status = pcl::io::saveOBJFile(filePath, *myCloud.mesh);
	return status == 0;
}

// There is no pcl::io::savePolygonFilePCD, so mesh can not be saved.
bool FileIO::savePCD(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat) {
	if (!myCloud.isValid) return false;
	string filePath = fileInfo.filePath().toLocal8Bit();
	int status = pcl::io::savePCDFile(filePath, *myCloud.cloud, isBinaryFormat);
	return status == 0;
}

bool FileIO::saveSTL(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat) {
	if (!myCloud.hasMesh) return false;
	string filePath = fileInfo.filePath().toLocal8Bit();
	return pcl::io::savePolygonFileSTL(filePath, *myCloud.mesh, isBinaryFormat);
}

bool FileIO::saveVTK(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat) {
	if (!myCloud.hasMesh) return false;
	string filePath = fileInfo.filePath().toLocal8Bit();
	return pcl::io::savePolygonFileVTK(filePath, *myCloud.mesh, isBinaryFormat);
}

bool FileIO::saveLAS(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat) {

	if (!myCloud.isValid) return false;
	string filePath = fileInfo.filePath().toLocal8Bit();

	LASwriteOpener laswriteopener;
	laswriteopener.set_file_name(filePath.c_str());

	// init header
	LASheader lasheader;
	lasheader.x_scale_factor = 0.0001;
	lasheader.y_scale_factor = 0.0001;
	lasheader.z_scale_factor = 0.0001;
	lasheader.x_offset = myCloud.cloudScale.x;
	lasheader.y_offset = myCloud.cloudScale.y;
	lasheader.z_offset = myCloud.cloudScale.z;
	lasheader.point_data_format = 1;
	lasheader.point_data_record_length = 28;

	// add a VLR with an empty payload that has only meaning to you and your users

	lasheader.add_vlr("my_one_VLR", 12345, 0, 0, FALSE, "this has no payload");

	// add a VLR with a small payload that has only meaning to you and your users

	U8* payload = new U8[64];
	memset(payload, 0, 64);
	strcpy((char*)payload, "this is a small payload followed by zeros");

	// note that LASheader takes over the memory control / deallocation for payload

	lasheader.add_vlr("my_other_VLR", 23456, 64, payload, FALSE, "this has a small payload");

	// init point 
	LASpoint laspoint;
	laspoint.init(&lasheader, lasheader.point_data_format, lasheader.point_data_record_length, 0);

	// open laswriter

	LASwriter* laswriter = laswriteopener.open(&lasheader);
	if (laswriter == 0)
	{
		fprintf(stderr, "ERROR: could not open laswriter\n");
	}
	// write points
	int count = myCloud.cloud->points.size();
	float radio = 1;
	if (myCloud.intensity.size() == count)
	{
		float max_ = *max_element(myCloud.intensity.begin(), myCloud.intensity.end());

		if (max_ <= 1)
		{
			radio = 255;
		}
	}

	for (int i = 0; i < count; i++)
	{
		// populate the point
		laspoint.set_X(10000 * myCloud.cloud->points[i].x);
		laspoint.set_Y(10000 * myCloud.cloud->points[i].y);
		laspoint.set_Z(10000 * myCloud.cloud->points[i].z);
		laspoint.rgb[0] = myCloud.cloud->points[i].r;
		laspoint.rgb[1] = myCloud.cloud->points[i].g;
		laspoint.rgb[2] = myCloud.cloud->points[i].b;
		//laspoint.set_R(myCloud.cloud->points[i].r);
		//laspoint.set_G(myCloud.cloud->points[i].g);
		//laspoint.set_B(myCloud.cloud->points[i].b);



		if (myCloud.intensity.size() == count)
			laspoint.set_intensity(myCloud.intensity[i] * radio);
		//laspoint.set_gps_time(0.0006 * i);

		// write the point
		laswriter->write_point(&laspoint);

		// add it to the inventory
		laswriter->update_inventory(&laspoint);
	}
	// update the header
	laswriter->update_header(&lasheader, TRUE);
	// close the writer
	I64 total_bytes = laswriter->close();

	delete laswriter;


	return true;
}

bool FileIO::save(const MyCloud& myCloud, const QFileInfo& fileInfo, bool isBinaryFormat) {
	string suffix = fromQString(fileInfo.suffix().toLower());
	if (suffix == "ply") {
		return savePLY(myCloud, fileInfo, isBinaryFormat);
	}
	else if (suffix == "obj") {
		return saveOBJ(myCloud, fileInfo);
	}
	else if (suffix == "pcd") {
		return savePCD(myCloud, fileInfo, isBinaryFormat);
	}
	else if (suffix == "stl") {
		return saveSTL(myCloud, fileInfo, isBinaryFormat);
	}
	else if (suffix == "vtk") {
		return saveVTK(myCloud, fileInfo, isBinaryFormat);
	}
	else if (suffix == "las") {

		return saveLAS(myCloud, fileInfo, isBinaryFormat);
	}
	else {
		return false;
	}
}

string FileIO::getInputFormatsStr() const {
	vector<string> suffixVec;
	vector<string> formats = { "placeholder" };
	for (auto it = inputFiltersMap.begin(); it != inputFiltersMap.end(); ++it) {
		suffixVec.push_back("*." + it->first);
		formats.push_back(it->second);
	}
	formats[0] = "All Supported Formats (" + joinStrVec(suffixVec, " ") + ")";
	return joinStrVec(formats, ";;");
}

string FileIO::getOutputFormatsStr() const {
	vector<string> formats;
	for (auto it = outputFiltersMap.begin(); it != outputFiltersMap.end(); ++it) {
		formats.push_back(it->second);
	}
	return joinStrVec(formats, ";;");
}
