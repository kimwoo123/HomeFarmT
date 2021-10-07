<template>
  <div class="home-schedule shadows">
    <div class="top-align">
      <span>오늘의 일정</span>
      <img src="@/assets/icons/history.svg" alt="history" class="history" @click="$router.push({ name: 'History' })">
    </div>
    <div class="schedule-box">
      <font-awesome-icon icon="chevron-left" class="icon" @click="previousSchedule"/>
      <div class="schedule-content" v-if="todaySchedule">
        <p class="title">{{ todaySchedule[idx].title }}</p>
        <p class="desc font-400">{{ todaySchedule[idx].desc }}로 감시를 실행합니다.</p>
        <p class="time font-400">{{todaySchedule[idx].time}} {{ todaySchedule[idx].isFinished }}</p>
      </div>
      <font-awesome-icon icon="chevron-right" class="icon" @click="nextSchedule"/>
    </div>
  </div>
</template>

<script>
import gql from 'graphql-tag'

let today = new Date()
let todayMonth = today.getMonth() + 1
let todayDay = today.getDate()
let todaytime = today.getHours() * 60 + today.getMinutes()

export default {
  name: 'HomeSchedule',
  data() {
    return {
      todaySchedule: '',
      idx: 0
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
          let d, t, tmp, AP, isFinished
          this.todaySchedule = []

          data.getSchedule.forEach(element => {
            let dt = element.schedule_time.split('T')

            d = dt[0].split('-')
            t = dt[1].split(':')

            if (todayMonth === parseInt(d[1]) && todayDay === parseInt(d[2]) && element.schedule_status === 'ON') {
              let tt = parseInt(t[0])

              if (tt == 12) {
                AP = '오후 '
              } else if (tt > 12) {
                AP = '오후 '
                t -= 12
              } else {
                AP = '오전 '
              }

              let ttt = parseInt(t[0]) * 60 + parseInt(t[1])
              if (todaytime > ttt) {
                isFinished = '완료'
              } else {
                isFinished = '예정'
              }

              tmp = {
                time: AP + dt[1],
                title: element.schedule_title,
                desc: element.schedule_desc,
                isFinished: isFinished
              }
              this.todaySchedule.push(tmp)
            }
          });
        },
    }
  },
  methods: {
    previousSchedule() {
      if (this.todaySchedule && 0 < this.idx) {
        this.idx -= 1
      }
    },
    nextSchedule() {
      if (this.todaySchedule && this.todaySchedule.length - 1 > this.idx) {
        this.idx += 1
      }
    }
  }
}
</script>

<style lang="scss" scoped>
  @import '@/components/Home/HomeSchedule.scss';
</style>