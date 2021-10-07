<template>
  <div :key="componentKey">
    <Navbar :title="'알림'" :left_icon="true" :right_text="''" :left_push="'Home'" :right_push="''"/>
    <div class="history-container">
      <div v-for="(date, index) in allDate" :key="index">
        <HistoryDayCard :history="allHistory" :date="date"/>
      </div>
    </div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import HistoryDayCard from '@/components/History/HistoryDayCard.vue'
import gql from 'graphql-tag'

export default {
  name: 'History',
  components: {
    Navbar,
    HistoryDayCard
  },
  data() {
    return {
      allHistory: [],
      allDate: new Set(),
      componentKey: 0,
    }
  },
  apollo: {
    getHistory: {
      query: gql`
        query {
          getHistory{
            historyid
            event_time
            event_title
            event_desc
            event_img
          }
        }`,
        error() {
          alert('로그인이 필요한 페이지입니다')
          this.$router.push({ name: 'Login' })
        },
        update(data) {
          this.allHistory = data.getHistory
          data.getHistory.forEach(element => {
              this.allDate.add(element.event_time.substring(0, 10))
          });
          this.componentKey += 1
        },
    }
  },
  methods: {
    gethi() {
      this.$apollo.queries.getHistory.refresh()
    },
    createHistory() {
      this.$apollo.mutate({
        mutation: gql`mutation ($time: String!, $title: String, $desc: String, $event_img: String) {
          createHistory(event_time: $time, event_title: $title, event_desc: $desc, event_img: $event_img) {
            event_time
            event_title
            event_desc
            event_img
          }
        }`,
        variables: {
          time: '2021-10-16T10:10',
          title: '침입자 감지',
          desc: '침입자를 감지했습니다',
          event_img: '이사진'
        }
        })
        .then((res) => {
          console.log(res.data)
        })
        .catch((err) => {
          console.log(err, 'no')
        })
    }
  }
}
</script>

<style lang="scss" scoped>
  @import "./History.scss";
</style>