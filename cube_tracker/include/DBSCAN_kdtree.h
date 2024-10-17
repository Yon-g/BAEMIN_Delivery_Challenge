#ifndef DBSCAN_KDTREE_H
#define DBSCAN_KDTREE_H

#include "DBSCAN_simple.h"
#include <pcl/point_types.h>
//클래스 템플릿에 인자를 전달해서 실제 코드를 생성하는 것을 클래스 템플릿 인스턴스화 라고 한다.
template <typename PointT> // 아래의 클래스에 대해 템플릿 정의, DBSCANKdtreeCluster 클래스에 대한 템플릿
class DBSCANKdtreeCluster: public DBSCANSimpleCluster<PointT> { // DBSCANKdtreeCluster 클래스는 DBSCANSimpleCluster 클래스를 상속받는다.
protected:
    virtual int radiusSearch (
        int index, double radius, std::vector<int> &k_indices,
        std::vector<float> &k_sqr_distances) const
    {
        return this->search_method_->radiusSearch(index, radius, k_indices, k_sqr_distances);
    }

}; // class DBSCANCluster

#endif // DBSCAN_KDTREE_H