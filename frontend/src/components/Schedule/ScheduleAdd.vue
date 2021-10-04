<template>
  <div>
    <!-- <b-form-datepicker id="datepicker-invalid" v-model="value" :state="false" v-if="!validate" class="mb-2"></b-form-datepicker>
    <b-form-datepicker menu-class="w-100" value-as-date placeholder="날짜를 선택해 주세요" id="datepicker-valid" v-model="value" v-if="validate"></b-form-datepicker> -->
    <!-- <b-form-input v-model="time" :id="`type-time`" type="time"></b-form-input>
    <b-form-input v-model="date" :id="`type-date`" type="date"></b-form-input>
    <b-form-input v-model="date" :id="`type-date`" type="datetime"></b-form-input> -->
    <b-form-input v-model="dateTime" type="datetime-local"></b-form-input>
    <button @click="setSchedule()">생성</button>
    <p>{{ dateTime }}</p>
  </div>
</template>

<script>
import gql from 'graphql-tag'

export default {
  data() {
    return {
      dateTime: '',
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

<style>

</style>