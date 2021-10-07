<template>
  <div>
    <Navbar :title="'마이 페이지'" :left_icon="true" :right_text="'로그아웃'" :left_push="'Home'" :right_push="''"/>
    <div class="my-container">
      <div class="input-container">
        <div class="item">
          <span>이메일</span>
          <input type="email" v-model="email" placeholder="이메일">
        </div>
        <div class="item">
          <span>비밀번호</span>
          <input type="password" v-model="password" placeholder="수정할 비밀번호">
        </div>
        <div class="item">
          <span>비밀번호 재입력</span>
          <input type="password" id="passwordConfirmation" v-model="passwordConfirmation" placeholder="비밀번호 재입력">
          <span>{{ errorMessage }}</span>
        </div>
      </div>

      <div class="bar"></div>

      <div class="input-container">
        <div class="item">
          <span>터틀봇 이름</span>
          <input type="text" v-model="turtlebotName" placeholder="터틀봇 이름">
        </div>
        <div class="item">
          <span>위치</span>
          <b-form-select class="select-form" v-model="region" :options="selectRegion"></b-form-select>
        </div>
      </div>

      <button class="my-btn" v-if="check" style="margin-top: 30px" @click="updateUserInfo()">확인</button>
      <button class="my-btn" v-else style="margin-top: 30px">불가</button>

    </div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'
import gql from 'graphql-tag'

export default {
  name: 'My',
  components: {
    Navbar,
  },
  data() {
    return {
      email: '',
      password: '',
      passwordConfirmation: '',
      turtlebotName: '',
      region: '대전',
      errorMessage: '',
      check: true,
      selectRegion: [
        { value : '대전', text: '대전' },
        { value : '서울', text: '서울' },
        { value : '광주', text: '광주' },
        { value : '구미', text: '구미' },
        { value : '부산', text: '부산' },
      ]
    }
  },
  apollo: {
    getUserInfo: {
      query: gql`
        query {
          findUser{
            email
            password
            hashid
            map
            route1
            route2
            route3
            region
            turtlebot
            patrol
          }
        }`, 
        update(data) {
          this.email = sessionStorage.getItem('email')
          this.turtlebotName = data.findUser.turtlebot
          this.region = data.findUser.region
        },
    }
  },
  methods: {
    updateUserInfo() {
      this.$apollo.mutate({
        mutation: gql`mutation ($password: String!, $turtlebot: String, $region: String) {
          updateUser(password: $password, turtlebot: $turtlebot, region: $region) {
            email
            region
            turtlebot
          }
        }`,
        variables: {
          password: this.password,
          turtlebot: this.turtlebotName,
          region: this.region
        }
        })
        .then((res) => {
          console.log(res.data)
        })
        .catch((err) => {
          console.log(err, 'no')
        })
    },
  },
  watch: {
    passwordConfirmation() {
      if (this.passwordConfirmation && (this.password === this.passwordConfirmation)) {
        this.errorMessage = '비밀번호가 일치합니다'
      } else {
        this.errorMessage = '비밀번호가 일치하지 않습니다' 
      }
      if (this.password === this.passwordConfirmation) {
        this.check = true
      } else if (this.password === '' && this.passwordConfirmation === '') {
          this.check = true
      } else {
         this.check = false
      }
    },
  },
}
</script>

<style lang="scss" scoped>
  @import './My.scss';
</style>