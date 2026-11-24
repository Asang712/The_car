(function() {
  var style = getComputedStyle(document.documentElement);
  var accent = style.getPropertyValue('--accent').trim();
  var accent2 = style.getPropertyValue('--accent2').trim();
  var ink = style.getPropertyValue('--ink').trim();
  var muted = style.getPropertyValue('--muted').trim();
  var rule = style.getPropertyValue('--rule').trim();
  var bg2 = style.getPropertyValue('--bg2').trim();

  // --- Chart: 坑的根因分类 ---
  var pitfallPie = echarts.init(document.getElementById('chart-pitfall-pie'), null, { renderer: 'svg' });
  pitfallPie.setOption({
    tooltip: {
      appendToBody: true,
      trigger: 'item',
      formatter: '{b}<br/>{c} 个坑 ({d}%)'
    },
    legend: {
      bottom: 0,
      left: 'center',
      textStyle: { color: muted, fontSize: 12 },
      itemGap: 16
    },
    color: [
      accent,
      accent2,
      accent + '99',
      accent2 + '99',
      muted,
      rule
    ],
    series: [{
      type: 'pie',
      radius: ['40%', '70%'],
      center: ['50%', '42%'],
      avoidLabelOverlap: true,
      itemStyle: {
        borderColor: '#fff',
        borderWidth: 2
      },
      label: {
        show: true,
        formatter: '{b}\n{c}',
        color: ink,
        fontSize: 12
      },
      labelLine: {
        show: true,
        length: 12,
        length2: 8,
        lineStyle: { color: rule }
      },
      data: [
        { value: 5, name: '硬件约束' },
        { value: 2, name: '启动前置条件' },
        { value: 1, name: 'ROS 2 Jazzy 版本' },
        { value: 1, name: 'QoS 不匹配' },
        { value: 1, name: '代码 bug' },
        { value: 1, name: '工具链体验' }
      ]
    }]
  });

  window.addEventListener('resize', function() { pitfallPie.resize(); });
})();
