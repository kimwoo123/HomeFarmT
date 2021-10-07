<template>
  <div>
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
      mutation: gql`
        mutation ($createScheduleScheduleTime: String!, $createScheduleScheduleTitle: String, $createScheduleScheduleDesc: String, $createScheduleScheduleStatus: String) {
                createSchedule(
                  schedule_time: $createScheduleScheduleTime, 
                  schedule_title: $createScheduleScheduleTitle, 
                  schedule_desc: $createScheduleScheduleDesc, 
                  schedule_status: $createScheduleScheduleStatus
                  ) {
                    scheduleid
                    schedule_time
                    schedule_title
                    schedule_desc
                    schedule_status
          }
        }
      `, 
      variables: {
        createScheduleScheduleTime: "19960221",
        createScheduleScheduleTitle: "물 주기",
        createScheduleScheduleDesc: "물물",
        createScheduleScheduleStatus: "ON"
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