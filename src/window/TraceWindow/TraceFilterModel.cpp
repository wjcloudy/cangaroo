#include "TraceFilterModel.h"

TraceFilterModel::TraceFilterModel(QObject *parent)
    : QSortFilterProxyModel{parent},
    _filterText("")
{
   setRecursiveFilteringEnabled(true);
}


void TraceFilterModel::setFilterText(QString filtertext)
{
    _filterText = filtertext;
}

bool TraceFilterModel::filterAcceptsRow(int source_row, const QModelIndex & source_parent) const
{
    // Pass all on no filter
    if(_filterText.length() == 0)
        return true;

    QModelIndex idx1 = sourceModel()->index(source_row, 3, source_parent); // CAN ID
    QModelIndex idx2 = sourceModel()->index(source_row, 4, source_parent); // Sender
    QModelIndex idx3 = sourceModel()->index(source_row, 5, source_parent); // Name

    QString datastr1 = sourceModel()->data(idx1).toString();
    QString datastr2 = sourceModel()->data(idx1).toString();
    QString datastr3 = sourceModel()->data(idx1).toString();

    fprintf(stderr, "Data for acceptance is %s\r\n", datastr1.toStdString().c_str());

    if( datastr1.contains(_filterText) ||
        datastr2.contains(_filterText) ||
        datastr3.contains(_filterText))
        return true;
    else
        return false;
}
