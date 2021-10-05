<template>
  <div>
    <Navbar :title="'마이 페이지'" :left_icon="true" :right_text="''" :left_push="'Home'" :right_push="''"/>
    <div class="my-container">

      <div class="input-container">
        <div class="item">
          <span>이메일</span>
          <input type="email" v-model="email" placeholder="이메일">
        </div>
        <div class="item">
          <span>비밀번호</span>
          <input type="password" v-model="password" @keyup.enter="requestLogin" placeholder="비밀번호">
        </div>
        <div class="item">
          <span>비밀번호 재입력</span>
          <input type="password" id="passwordConfirmation" v-model="passwordConfirmation" placeholder="비밀번호 재입력">
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
          <input type="text" v-model="region" placeholder="위치">
        </div>
      </div>

      <button class="my-btn" style="margin-top: 30px" @click="submit()">확인</button>

    </div>
  </div>
</template>

<script>
import Navbar from '@/components/common/Navbar.vue'

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
      region: '',
    }
  },
  methods: {
    submit() {
      
    }
  }
}
</script>

<style lang="scss" scoped>
  @import './My.scss';
</style>