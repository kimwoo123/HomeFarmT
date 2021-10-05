<template>
  <div>
    <Navbar :title="'10월 스케줄'" :left_icon="true" :right_text="''" :left_push="'Schedule'" :right_push="''" />
    <div class="ScheduleNew-container">
      <b-form-input v-model="dateTime" type="datetime-local"></b-form-input>
      <p>{{ dateTime }}</p>

      <p class="title">스케줄</p>
      <div class="schedule">
        <button class="item shadows" @click="title='감시하기'" :class="{'item-active' : title==='감시하기'}">감시하기</button>
        <button class="item shadows" @click="title='제어하기'" :class="{'item-active' : title==='제어하기'}">제어하기</button>
      </div>

      <div v-if="title==='감시하기'" class="box">
        <p class="title">경로 선택</p>
        <div class="path shadows">
          <font-awesome-icon icon="chevron-left"/>
          <span>1번 경로</span>
          <font-awesome-icon icon="chevron-right"/>
        </div>
      </div>

      <div v-if="title==='제어하기'" class="box">
        <p class="title">기기 선택</p>
        <div class="control shadows">

          <img src="@/assets/icons/devices/light.svg" alt="light">
          
          <div class="text-group">
            <span class="name">형광등</span>
            <span class="position">부엌</span>
          </div>
          
          <img src="@/assets/icons/switch.svg" alt="switch" class="switch">
        </div>
      </div>

      <div v-if="!title" class="box"></div>

      <button class="my-btn" @click="setSchedule()">스케줄 추가</button>
    </div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import gql from 'graphql-tag'

export default {
  name: 'ScheduleNew',
  components: {
    Navbar,
  },
  data() {
    return {
      dateTime: '',
      title: '',
      description: '',
      status: true,
    }
  },
  methods: {
    setSchedule() {
    this.$apollo.mutate({
      mutation: gql`mutation ($schedule_time: String!) {
        createSchedule(schedule_time: $schedule_time) {
          schedule_time
        }
      }`, 
      variables: {
        schedule_time: this.dateTime
      }
      })
      .then((res) => {
        console.log(res, 'done')
        window.location.reload()
      })
      .catch((err) => {
        console.log(err, 'no')
      })
    }
  }
}
</script>

<style lang="scss" scoped>
  @import './ScheduleNew.scss';
</style>