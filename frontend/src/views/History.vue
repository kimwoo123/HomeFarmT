<template>
  <div>
    <Navbar :title="'알림'" :left_icon="true" :right_text="''" :left_push="'Home'" :right_push="''"/>
    <div class="history-container">
      <HistoryDayCard/>
      <button @click="createHistory()">실험</button>

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
      data: [
        {
          data: '날짜시간형식',
          time: '10: 30',
          title: '침입자 감지',
          description: 'OO방에서 침입자를 감지했습니다.',
          pic: require('@/assets/images/cam.png')
        },
        {
          data: '날짜시간형식',
          time: '9: 30',
          title: '침입자 감지',
          description: 'OO방에서 침입자를 감지했습니다.',
          pic: require('@/assets/images/cam.png')
        },
        {
          data: '날짜시간형식',
          time: '11: 30',
          title: '침입자 감지',
          description: 'OO방에서 침입자를 감지했습니다.',
          pic: ''
        }
      ]
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
        update(data) {
          console.log(data)
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
          time: '되니',
          title: '될거야',
          desc: '됩니다',
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