<template>
  <div>
    <Navbar :title="'10월 스케줄'" :left_icon="true" :right_text="'추가'" :left_push="'Home'" :right_push="'ScheduleNew'" />
    <div class="schedule-container">
      <ScheduleCalendar @set-date="setDate"/>
      <ScheduleCard v-for="(schedule, idx) in selectDaySchedule" :key="idx" :schedule="schedule" :idx="idx"/>
    </div>
    <!-- <SchedulePlan/> -->
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import ScheduleCalendar from '@/components/Schedule/ScheduleCalendar.vue'
// import SchedulePlan from '@/components/Schedule/SchedulePlan.vue'
import ScheduleCard from '@/components/Schedule/ScheduleCard.vue'
import gql from 'graphql-tag'

let today = new Date()
let todayGetDate = today.getDate()

export default {
  name: 'Schedule',
  components: {
    Navbar,
    ScheduleCalendar,
    // SchedulePlan,
    ScheduleCard
  },
  data() {
    return {
      allSchedule: [],
      selectDay: todayGetDate,
      selectDaySchedule: [],
    }
  },
  apollo: {
    getSchedule: {
      query: gql`
        query {
          getSchedule{
            scheduleid
            schedule_time
            schedule_title
            schedule_desc
            schedule_status
          }
        }`,
        update(data) {
          let d, t, tmp, AP, s

          this.allSchedule = data.getSchedule

          data.getSchedule.forEach(element => {
            let dt = element.schedule_time.split('T')
            d = dt[0].split('-')
            t = dt[1]
            
            // 여기에 parseInt(d[1]) === this.selectMonth (몇월 선택했는지) 넣어주면 됨
            if (parseInt(d[2]) === this.selectDay) {
              let tt = parseInt(t.split(':')[0])

              if (tt == 12) {
                AP = '오후 '
              } else if (tt > 12) {
                AP = '오후 '
                t -= 12
              } else {
                AP = '오전 '
              }

              if (element.schedule_status === 'ON') {
                s = 1
              } else {
                s = 0
              }

              tmp = {
                date: d[0] + '년 ' + d[1] + '월 ' + d[2] + '일',
                time: AP + t,
                title: element.schedule_title,
                desc: element.schedule_desc,
                status: s
              }
              this.selectDaySchedule.push(tmp)
            }
            console.log(this.selectDaySchedule)

          });
        },
    }
  },
  methods: {
    setDate(date) {
      this.selectDay = date
      let d, t, tmp, AP

      this.selectDaySchedule = []
      this.allSchedule.forEach(element => {
        let dt = element.schedule_time.split('T')
        d = dt[0].split('-')
        t = dt[1]
        
        if (parseInt(d[2]) === this.selectDay) {
          let tt = parseInt(t.split(':')[0])
          
          if (tt == 12) {
            AP = '오후 '
          } else if (tt > 12) {
            AP = '오후 '
            t -= 12
          } else {
            AP = '오전 '
          }

          tmp = {
            date: d[0] + '년 ' + d[1] + '월 ' + d[2] + '일',
            time: AP + t,
            title: element.schedule_title,
            desc: element.schedule_desc,
            status: element.schedule_status
          }
          this.selectDaySchedule.push(tmp)
        }
      })
    }
  }
}
</script>

<style lang="scss" scoped>
  @import './Schedule.scss';
</style>