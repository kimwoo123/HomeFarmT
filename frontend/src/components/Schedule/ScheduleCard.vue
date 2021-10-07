<template>
  <div class="card-box shadows">

    <div class="content-box">
      <div class="content">
        <span class="title">{{ schedule.title }}</span>
        <span class="description font-300">{{ schedule.desc }}로 감시를 실행합니다.</span>
      </div>
      <SwitchBtn class="switch-align" @is-checked="changeStatus" :defaultValue="schedule.status" :checkboxId="idx"/>
    </div>
    <div class="time-box">
      <img src="@/assets/icons/calender.svg" alt="date" class="icon">
      <span class="date">{{ schedule.date }}</span>
      <img src="@/assets/icons/time.svg" alt="time" class="icon">
      <span class="time">{{ schedule.time }}</span>
    </div>

  </div>
</template>

<script>
import SwitchBtn from '@/components/common/SwitchBtn.vue'
import gql from 'graphql-tag'

export default {
  name: 'ScheduleCard',
  components: {
    SwitchBtn
  },
  props: {
    schedule: Object,
    idx: Number
  },
  methods: {
    changeStatus(isChecked) {
      this.$apollo.mutate({
        mutation: gql`mutation ($updateScheduleStatus: String $updateScheduleId: Int) {
          updateScheduleStatus(
            scheduleid: $updateScheduleId
            schedule_status: $updateScheduleStatus
            ) {
              scheduleid
              schedule_status
            }
          }
        `, 
        variables: {
          updateScheduleId: this.schedule.id,
          updateScheduleStatus: isChecked ? 'ON' : 'OFF'
        }
        })
        .then((res) => {
          console.log(res.data.updateScheduleStatus.schedule_status)
        })
        .catch((err) => {
          console.log(err, 'no')
        })
    }
  }
}
</script>

<style lang="scss" scoped>
  @import '@/components/Schedule/ScheduleCard.scss';
</style>
