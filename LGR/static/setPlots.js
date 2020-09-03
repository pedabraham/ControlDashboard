function setPlot(features){
  var Data = features.Yset;
  var XData = features.Xset;
  var sets = [];
    for (var ids = 0; ids < Data.length; ids++){
      let dataSet = Data[ids];
      let Xset = XData[ids];
      var dataWFormat = [];
      for (var i = 0; i < dataSet.length; i++) {
        var obj = {x:Xset[i],y:dataSet[i]};
        dataWFormat.push(obj);
      }
      var color = features.color;
      if(ids>0){
        color = 'rgb('+(90*ids).toString()+','+ (100*ids).toString()+','+ (70*ids).toString()+')';
      }
      var plot = {
        data: dataWFormat,
        label: features.label,
        borderColor: color,
        backgroundColor: color,
        fill: false
      };
      sets.push(plot);
    }

  return{
      type : 'line',
      data : {
        datasets: sets
      },
      options:{
        responsive: true,
        title:{
          display: true,
          text: features.label
        },
        scales: {
            xAxes: [{
                type: 'linear',
                position: 'bottom'
            }]
        }
      }
    };
}
