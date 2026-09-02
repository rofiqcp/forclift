// Extracted from agv_gui.cpp for maintainability.
class LivePlotWidget:public QWidget{
  public:explicit LivePlotWidget(QString title,int maxPoints=300,QWidget*p=nullptr):QWidget(p),title_(std::move(title)),maxPoints_(maxPoints){
    setMinimumHeight(210);
    setSizePolicy(QSizePolicy::Expanding,QSizePolicy::Expanding);
  }
  void append(const QMap<QString,double>&v){
    for(auto it=v.cbegin();
    it!=v.cend();
    ++it){
      auto&d=data_[it.key()];
      d.push_back(it.value());
      while(d.size()>maxPoints_)d.pop_front();
    }
    update();
  }
  void clear(){
    data_.clear();
    update();
  }
  protected:void paintEvent(QPaintEvent*)override{
    QPainter p(this);
    p.fillRect(rect(),QColor(kPanel));
    p.setRenderHint(QPainter::Antialiasing);
    p.setPen(QColor(kText));
    p.drawText(QRect(10,6,width()-20,24),Qt::AlignLeft|Qt::AlignVCenter,title_);
    QRectF r(48,34,width()-64,height()-58);
    p.setPen(QPen(QColor("#343a44"),1));
    p.drawRect(r);
    if(data_.isEmpty())return;
    double mn=1e100,mx=-1e100;
    for(const auto&d:data_)for(double x:d)if(std::isfinite(x)){
      mn=std::min(mn,x);
      mx=std::max(mx,x);
    }
    if(!(mn<1e90)){
      return;
    }
    if(std::abs(mx-mn)<1e-9){
      mn-=1;
      mx+=1;
    }
    const QVector<QColor> cols={
      QColor(kGold),QColor(kBlue),QColor(kGreen),QColor(kOrange),QColor(kRed),QColor("#b47cff")
    };
    int ci=0;
    int legendY=8;
    for(auto it=data_.cbegin();
    it!=data_.cend();
    ++it,++ci){
      const auto&d=it.value();
      if(d.size()<2)continue;
      QPainterPath path;
      bool first=true;
      for(int i=0;
      i<d.size();
      ++i){
        double v=d[i];
        if(!std::isfinite(v))continue;
        double x=r.left()+r.width()*double(i)/std::max(1,d.size()-1);
        double y=r.bottom()-r.height()*(v-mn)/(mx-mn);
        if(first){
          path.moveTo(x,y);
          first=false;
        }
        else path.lineTo(x,y);
      }
      p.setPen(QPen(cols[ci%cols.size()],1.8));
      p.drawPath(path);
      p.drawText(width()-145,legendY+15*ci,it.key());
    }
    p.setPen(QColor(kMuted));
    p.drawText(4,int(r.top()+10),QString::number(mx,'g',4));
    p.drawText(4,int(r.bottom()),QString::number(mn,'g',4));
  }
  private:QString title_;
  int maxPoints_;
  QMap<QString,QVector<double>>data_;
};
class ReportPlotWidget : public QWidget {
  public:
  explicit ReportPlotWidget(QWidget *parent = nullptr) : QWidget(parent) {
    setMinimumHeight(280);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  }
  void setPlot(const QString &title, const QString &xLabel, const QString &yLabel,
  const QMap<QString, QVector<QPointF>> &series, bool connectPoints = true,
  const QString &emptyMessage = QString()) {
    title_ = title;
    xLabel_ = xLabel;
    yLabel_ = yLabel;
    series_ = series;
    connectPoints_ = connectPoints;
    emptyMessage_ = emptyMessage;
    update();
  }
  void clear() {
    series_.clear();
    update();
  }
  QString emptyMessageForCurrentSeries(const QString &defaultMessage = QString()) const {
    if (!series_.isEmpty()) return QString();
    if (!emptyMessage_.isEmpty()) return emptyMessage_;
    return defaultMessage;
  }
  protected:
  void paintEvent(QPaintEvent *) override {
    QPainter painter(this);
    painter.fillRect(rect(), QColor(kPanel));
    painter.setRenderHint(QPainter::Antialiasing);
    painter.setPen(QColor(kText));
    painter.setFont(QFont(painter.font().family(), 10, QFont::DemiBold));
    painter.drawText(QRect(12, 6, width() - 24, 26), Qt::AlignLeft | Qt::AlignVCenter,
    title_.isEmpty() ? QStringLiteral("Grafik pengujian") : title_);
    const QRectF plot(64, 40, std::max(80, width() - 220), std::max(80, height() - 98));
    painter.setPen(QPen(QColor(QStringLiteral("#343a44")), 1));
    painter.drawRect(plot);
    double minX = std::numeric_limits<double>::infinity();
    double maxX = -std::numeric_limits<double>::infinity();
    double minY = std::numeric_limits<double>::infinity();
    double maxY = -std::numeric_limits<double>::infinity();
    for (auto it = series_.cbegin();
    it != series_.cend();
    ++it) {
      for (const QPointF &point : it.value()) {
        if (!std::isfinite(point.x()) || !std::isfinite(point.y())) continue;
        minX = std::min(minX, point.x());
        maxX = std::max(maxX, point.x());
        minY = std::min(minY, point.y());
        maxY = std::max(maxY, point.y());
      }
    }
    if (!std::isfinite(minX) || !std::isfinite(minY)) {
      painter.setPen(QColor(QStringLiteral("#e7953f")));
      painter.setFont(QFont(painter.font().family(), 10, QFont::DemiBold));
      const QString message = emptyMessage_.isEmpty()
        ? QStringLiteral("BELUM ADA DATA\nMenunggu sumber telemetry untuk grafik ini.")
        : emptyMessage_;
      painter.drawText(plot.adjusted(18, 18, -18, -18),
                       Qt::AlignCenter | Qt::TextWordWrap, message);
      return;
    }
    if (std::abs(maxX - minX) < 1.0e-9) {
      minX -= 1.0;
      maxX += 1.0;
    }
    if (std::abs(maxY - minY) < 1.0e-9) {
      minY -= 1.0;
      maxY += 1.0;
    }
    const double yMargin = 0.08 * (maxY - minY);
    minY -= yMargin;
    maxY += yMargin;
    painter.setFont(QFont(painter.font().family(), 8));
    for (int tick = 0;
    tick <= 5;
    ++tick) {
      const double ratio = tick / 5.0;
      const double x = plot.left() + ratio * plot.width();
      const double y = plot.bottom() - ratio * plot.height();
      painter.setPen(QPen(QColor(QStringLiteral("#292e36")), 1, Qt::DashLine));
      painter.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
      painter.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
      painter.setPen(QColor(kMuted));
      painter.drawText(QRectF(x - 35, plot.bottom() + 5, 70, 18), Qt::AlignHCenter,
      QString::number(minX + ratio * (maxX - minX), 'g', 4));
      painter.drawText(QRectF(2, y - 9, 56, 18), Qt::AlignRight | Qt::AlignVCenter,
      QString::number(minY + ratio * (maxY - minY), 'g', 4));
    }
    painter.setPen(QColor(kMuted));
    painter.drawText(QRectF(plot.left(), height() - 28, plot.width(), 18), Qt::AlignCenter, xLabel_);
    painter.save();
    painter.translate(17, plot.center().y());
    painter.rotate(-90.0);
    painter.drawText(QRectF(-plot.height() / 2.0, -10, plot.height(), 18), Qt::AlignCenter, yLabel_);
    painter.restore();
    const QVector<QColor> colors = {
      QColor(kGold), QColor(kBlue), QColor(kGreen),
      QColor(kOrange), QColor(kRed), QColor(QStringLiteral("#b47cff")), QColor(QStringLiteral("#5ed4c7"))
    };
    int seriesIndex = 0;
    int legendY = 44;
    for (auto it = series_.cbegin();
    it != series_.cend();
    ++it, ++seriesIndex) {
      const QColor color = colors[seriesIndex % colors.size()];
      QPainterPath path;
      bool first = true;
      for (const QPointF &point : it.value()) {
        if (!std::isfinite(point.x()) || !std::isfinite(point.y())) continue;
        const QPointF screen(
        plot.left() + (point.x() - minX) / (maxX - minX) * plot.width(),
        plot.bottom() - (point.y() - minY) / (maxY - minY) * plot.height());
        if (first) {
          path.moveTo(screen);
          first = false;
        }
        else if (connectPoints_) path.lineTo(screen);
        else path.moveTo(screen);
        painter.setPen(Qt::NoPen);
        painter.setBrush(color);
        painter.drawEllipse(screen, 2.7, 2.7);
      }
      if (connectPoints_ && !path.isEmpty()) {
        painter.setPen(QPen(color, 1.8));
        painter.setBrush(Qt::NoBrush);
        painter.drawPath(path);
      }
      painter.setPen(QPen(color, 3));
      painter.drawLine(QPointF(plot.right() + 16.0, legendY + 6.0), QPointF(plot.right() + 34.0, legendY + 6.0));
      painter.setPen(QColor(kText));
      painter.drawText(QRectF(plot.right() + 40, legendY - 3, 115, 18), Qt::AlignLeft | Qt::AlignVCenter, it.key());
      legendY += 20;
    }
  }
  private:
  QString title_, xLabel_, yLabel_;
  QMap<QString, QVector<QPointF>> series_;
  QString emptyMessage_;
  bool connectPoints_{
    true
  };
};
class ReportManager{
  public:ReportManager(std::shared_ptr<YamlStore>gui,QMap<QString,std::shared_ptr<YamlStore>>stores):gui_(std::move(gui)),stores_(std::move(stores)){
  }
  QString root()const{
    QString p=gui_->get("reporting.output_directory",QStringLiteral("~/.ros/agv_gui_reports")).toString();
    if(p.startsWith('~'))p=QDir::homePath()+p.mid(1);
    QDir().mkpath(p);
    return p;
  }
  QString saveCsv(const QString&stem,const QVector<QVariantMap>&rows){
    const QString path=QDir(root()).filePath(stem+"_"+nowStamp()+".csv");
    QSet<QString>keys;
    for(const auto&r:rows)for(auto it=r.cbegin();
    it!=r.cend();
    ++it)keys.insert(it.key());
    QStringList headers=keys.values();
    std::sort(headers.begin(),headers.end());
    QSaveFile f(path);
    if(!f.open(QIODevice::WriteOnly|QIODevice::Text))return {
    };
    QTextStream ts(&f);
    for(int i=0;
    i<headers.size();
    ++i){
      if(i)ts<<',';
      ts<<csvEscape(headers[i]);
    }
    ts<<'\n';
    for(const auto&r:rows){
      for(int i=0;
      i<headers.size();
      ++i){
        if(i)ts<<',';
        QVariant v=r.value(headers[i]);
        QString text;
        if(v.userType()==QMetaType::QVariantList||v.userType()==QMetaType::QVariantMap)text=variantJsonText(v);
        else text=v.toString();
        ts<<csvEscape(text);
      }
      ts<<'\n';
    }
    if(!f.commit())return {
    };
    writeMeta(path,rows);
    return path;
  }
  QString savePng(const QString&stem,QWidget*w){
    const QString path=QDir(root()).filePath(stem+"_"+nowStamp()+".png");
    return w->grab().save(path)?path:QString();
  }
  QString saveCsvOrdered(const QString &stem, const QStringList &headers, const QVector<QVariantMap> &rows) {
    const QString path = QDir(root()).filePath(stem + "_" + nowStamp() + ".csv");
    QSaveFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) return {
    };
    QTextStream stream(&file);
    for (int column = 0;
    column < headers.size();
    ++column) {
      if (column) stream << ',';
      stream << csvEscape(headers[column]);
    }
    stream << '\n';
    for (const QVariantMap &row : rows) {
      for (int column = 0;
      column < headers.size();
      ++column) {
        if (column) stream << ',';
        const QVariant value = row.value(headers[column]);
        stream << csvEscape((value.userType() == QMetaType::QVariantList || value.userType() == QMetaType::QVariantMap)
        ? variantJsonText(value) : value.toString());
      }
      stream << '\n';
    }
    if (!file.commit()) return {
    };
    writeMeta(path, rows);
    return path;
  }
  private:std::shared_ptr<YamlStore>gui_;
  QMap<QString,std::shared_ptr<YamlStore>>stores_;
  QJsonObject numericSummary(const QVector<QVariantMap>&rows)const{
    QSet<QString> keys;
    for(const auto&r:rows)for(auto it=r.cbegin();
    it!=r.cend();
    ++it)keys.insert(it.key());
    QJsonObject out;
    for(const QString&k:keys){
      QVector<double> values;
      for(const auto&r:rows){
        bool ok=false;
        double v=r.value(k).toDouble(&ok);
        if(ok&&std::isfinite(v))values<<v;
      }
      if(values.isEmpty())continue;
      double sum=std::accumulate(values.begin(),values.end(),0.0),mean=sum/values.size(),sq=0.0,absSum=0.0,maxAbs=0.0;
      for(double v:values){
        sq+=(v-mean)*(v-mean);
        absSum+=std::abs(v);
        maxAbs=std::max(maxAbs,std::abs(v));
      }
      QJsonObject st{
        {
          "count",values.size()
        },{
          "mean",mean
        },{
          "min",*std::min_element(values.begin(),values.end())
        },{
          "max",*std::max_element(values.begin(),values.end())
        },{
          "stddev",std::sqrt(sq/values.size())
        }
      };
      const QString low=k.toLower();
      if(low.contains("error")||low.contains("residual")){
        double e2=0;
        for(double v:values)e2+=v*v;
        st["rmse"]=std::sqrt(e2/values.size());
        st["mean_abs"]=absSum/values.size();
        st["max_abs"]=maxAbs;
      }
      out[k]=st;
    }
    return out;
  }
  void writeMeta(const QString&csv,const QVector<QVariantMap>&rows){
    QJsonObject meta;
    meta["created_at"]=QDateTime::currentDateTime().toString(Qt::ISODate);
    meta["csv"]=QFileInfo(csv).fileName();
    meta["row_count"]=rows.size();
    meta["numeric_summary"]=numericSummary(rows);
    for(const QString &k:{
      QStringLiteral("project_name"),QStringLiteral("operator"),QStringLiteral("session_id"),QStringLiteral("experiment_category"),QStringLiteral("experiment_variant"),QStringLiteral("experiment_notes")
    })meta[k]=gui_->get("reporting."+k).toString();
    QJsonObject cfg;
    for(auto it=stores_.cbegin();
    it!=stores_.cend();
    ++it){
      it.value()->reload();
      QJsonObject f;
      const QByteArray raw=it.value()->raw();
      f["path"]=it.value()->path();
      f["sha256"]=QString::fromLatin1(QCryptographicHash::hash(raw,QCryptographicHash::Sha256).toHex());
      cfg[it.key()]=f;
    }
    meta["configuration"]=cfg;
    QSaveFile f(csv.left(csv.size()-4)+".meta.json");
    if(f.open(QIODevice::WriteOnly)){
      f.write(QJsonDocument(meta).toJson(QJsonDocument::Indented));
      f.commit();
    }
  }
};
