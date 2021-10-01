<template>
  <div :key="componentKey">
    <div class="schedule-header">
      <span class="schedule-text">Schedule</span>
      <button
      @click="add"
      ><font-awesome-icon icon="plus" /><span> 추가</span></button>
    </div>
    <component :is="buttons"></component>
    <div class="schedule">
      <div v-for="time in 25" :key=time>
        <div v-if="time <= 10" class="time-box">
          0{{ time-1 }}:00
          <div v-if="todaySchedule['0' + time]">
            순찰 예정
          </div>
        </div>
        <div v-else class="time-box">
          {{ time-1 }}:00
          <div v-if="todaySchedule[time]">
            순찰 예정
          </div>
        </div>
      </div>
      <div class="item">

      </div>
    </div>
  </div>
</template>

<style lang="scss" scoped>
  @import './SchedulePlan.scss';
</style>

<script>
import gql from 'graphql-tag'

const AddSchedule = () => import('./ScheduleAdd.vue')
const timezoneOffset = new Date().getTimezoneOffset() * 60 * 1000

export default {
  name: 'SchedulePlan',
  props: [ 'date' ],
    data() {
    return {
      buttons: '',
      userSchedule: [],
      todaySchedule: {},
      componentKey: 0,
    }
  },
  created() {
    this.$apollo.queries.getSchedule.refresh()
  },
  methods: {
    add() {
      this.buttons = AddSchedule
    }
  },
  apollo: {
    getSchedule: {
      query: gql`
        query {
          getSchedule{
            schedule_time
          }
        }`,
        update(data) {
          const todayDateTmp = new Date(this.date - timezoneOffset).toISOString().substring(0, 10)
          for (let schedule of data.getSchedule) {
            this.userSchedule.push(schedule.schedule_time)
            if (todayDateTmp === schedule.schedule_time.substring(0, 10)) {
              this.todaySchedule[schedule.schedule_time.substring(11, 13)] = schedule.schedule_time
            }
          }
          this.componentKey += 1;
        },
    }
  },
  watch: {
    date: {
      immediate: true,
      handler() {
        let dailySchedule = {}
        if (this.userSchedule) {
          const todayDate = new Date(this.date - timezoneOffset).toISOString().substring(0, 10)
          for (let schedule of this.userSchedule) {
            if (todayDate === schedule.substring(0, 10)) {
              dailySchedule[schedule.substring(11, 13)] = schedule
            }
          }
        }
        this.todaySchedule = dailySchedule
        this.componentKey += 1;
      },
    }
  }
}
</script>