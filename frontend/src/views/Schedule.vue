<template>
  <div>
    <ScheduleCalendar/>
    <!-- <SchedulePlan/> -->
  </div>
</template>

<style lang="scss" scoped>
  @import '@/views/Schedule.scss';
</style>

<script>
import ScheduleCalendar from '../components/Schedule/ScheduleCalendar.vue'
// import SchedulePlan from '../components/Schedule/SchedulePlan.vue'


export default {
  name: 'Schedule',
  components: {
    ScheduleCalendar,
    // SchedulePlan
  },
}
</script>