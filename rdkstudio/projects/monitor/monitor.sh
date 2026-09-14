#!/usr/bin/env bash
# Lightweight RDK S100 runtime monitor: CPU / MEM / TEMP / BPU / TOP
# Usage: ./monitor.sh [interval_seconds]   (default 1s)   Ctrl-C to stop
set -u
INT="${1:-1}"

# snapshot /proc/stat lines: cpu (total) + cpu0..cpuN
cpu_snap() {
  awk '/^cpu[0-9]* / {printf "%s", $0 "\n"}' /proc/stat
}

bar() {
  local p="$1" w=30 n i
  n=$(( p*w/100 ))
  printf "["
  for ((i=0;i<w;i++)); do [ $i -lt $n ] && printf '#' || printf ' '; done
  printf "] %3d%%" "$p"
}

PREV=$(cpu_snap)
while true; do
  sleep "$INT"
  NOW=$(cpu_snap)
  # busy% per line: busy = (user+nice+sys+irq+softirq+steal) delta / total delta
  ROWS=$(awk -v a="$PREV" -v b="$NOW" '
    function linebusy(s,  n,x,i,sum,idle){ n=split(s,x," ");
      sum=0; idle=0;
      for(i=2;i<=9;i++){ sum+=x[i] }
      idle=x[5]+x[6];           # idle + iowait
      return sum " " idle }
    BEGIN{
      na=split(a,AA,"\n"); nb=split(b,BB,"\n");
      for(l=1;l<=na;l++){
        split(AA[l],xa," "); split(BB[l],xb," ");
        split(linebusy(AA[l]),pa," "); split(linebusy(BB[l]),pb," ");
        dt=pb[1]-pa[1]; di=pb[2]-pa[2];
        p = (dt>0) ? (dt-di)*100/dt : 0;
        printf "%s %.0f\n", xa[1], p;
      }
    }')
  PREV="$NOW"

  CPU_PCT=$(echo "$ROWS" | awk '$1=="cpu"{print $2}')
  CORE_PCTS=$(echo "$ROWS" | awk '$1!="cpu"{printf "%s=%d%% ", substr($1,4), $2}')
  MEM=$(free -m | awk '/^Mem:/{printf "used %dMB / %dMB (%.0f%%)", $3, $2, $3*100/$2}')
  TZ=""
  for z in /sys/class/thermal/thermal_zone*; do
    t=$(cat "$z/temp" 2>/dev/null); [ -z "$t" ] && continue
    TZ+="$(basename $z)=$(awk -v t=$t 'BEGIN{printf "%.1f", t/1000}')C "
  done
  BPU=/sys/devices/system/bpu/bpu0
  BPU_RATIO=$(cat $BPU/ratio 2>/dev/null)
  BPU_PWR=$(cat $BPU/power_level 2>/dev/null)
  BPU_TASKS=$(awk '!/us$/ && NF>=6 && $6 ~ /^[0-9]+$/{c++} END{print c+0}' $BPU/task_time 2>/dev/null)
  TS=$(date '+%H:%M:%S')

  echo "===================== $TS ====================="
  printf "CPU total: "; bar "$CPU_PCT"; echo
  printf "CPU cores: %s\n" "$CORE_PCTS"
  printf "MEM : %s\n" "$MEM"
  printf "TEMP: %s\n" "$TZ"
  printf "BPU : ratio=%s%%  power_level=%s  running_tasks=%s\n" "$BPU_RATIO" "$BPU_PWR" "$BPU_TASKS"
  echo "-- TOP CPU processes --"
  ps -eo pid,user,pcpu,pmem,comm --sort=-pcpu | head -n 6
done
