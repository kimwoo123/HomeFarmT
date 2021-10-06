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
// mutation (
//         $schedule_time: String!,
//         $schedule_title: String,
//         $schedule_desc: String,
//         $schedule_status: String,
//         ) 
//         {
//         createSchedule(
//           schedule_time: $schedule_time,
//           schedule_title: $schedule_title,
//           schedule_desc: $schedule_desc,
//           schedule_status: $schedule_status, 
//           ) 
//           {
//           schedule_time
//           schedule_title
//           schedule_desc
//           schedule_status
//         }
//       }
        // mutation ($createScheduleScheduleTime: String!, $createScheduleScheduleTitle: String, $createScheduleScheduleDesc: String, $createScheduleScheduleStatus: String) {
        // schedule_title: $createScheduleScheduleTitle, 
        // schedule_desc: $createScheduleScheduleDesc, 
        // schedule_status: $createScheduleScheduleStatus

                  //         schedule_time
                  // schedule_title
                  // schedule_desc
                  // schedule_status