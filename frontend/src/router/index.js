import Vue from 'vue'
import VueRouter from 'vue-router'
import Signup from '../views/user/Signup.vue'
import Login from '../views/user/Login.vue'
import My from '../views/user/My.vue'
import Home from '../views/Home.vue'
import History from '../views/History.vue'

Vue.use(VueRouter)

const routes = [
  {
    path: '*',
    redirect: '/'
  },
  {
    path: '/',
    redirect: Home,
  },
  {
    path: '/signup',
    name: 'Signup',
    component: Signup,
    meta: {
      title: '으라차차 회원가입'
    }
  },
  {
    path: '/login',
    name: 'Login',
    component: Login,
    meta: {
      title: '으라차차 로그인'
    }
  },
  {
    path: '/my',
    name: 'My',
    component: My,
    meta: {
      title: '으라차차 회원정보'
    }
  },
  {
    path: '/home',
    name: 'Home',
    component: Home,
    meta: {
      title: '으라차차!'
    }
  },
  // iot
  {
    path: '/iot',
    name: 'IoT',
    component: () => import('../views/IoT/IoT.vue'),
    meta: {
      title: '으라차차 IoT'
    }
  },
  {
    path: '/iot/new',
    name: 'IoTNew',
    component: () => import('../views/IoT/IoTNew.vue'),
    meta: {
      title: '으라차차 IoTNew'
    }
  },
  // schedule
  {
    path: '/schedule',
    name: 'Schedule',
    component: () => import('../views/Schedule/Schedule.vue'),
    meta: {
      title: '으라차차 일정'
    }
  },
  {
    path: '/schedule/new',
    name: 'ScheduleNew',
    component: () => import('../views/Schedule/ScheduleNew.vue'),
    meta: {
      title: '으라차차 일정'
    }
  },
  // patrol
  {
    path: '/patrol',
    name: 'Patrol',
    component: () => import('../views/Patrol/Patrol.vue'),
    meta: {
      title: '으라차차 순찰'
    }
  },
  {
    path: '/patrol/setting',
    name: 'PatrolSetting',
    component: () => import('../views/Patrol/PatrolSetting.vue'),
    meta: {
      title: '으라차차 순찰'
    }
  },
  // control 
  {
    path: '/control',
    name: 'Control',
    component: () => import('../views/Control/Control.vue'),
    meta: {
      title: '으라차차 조작'
    }
  },
  // history
  {
    path: '/history',
    name: 'History',
    component: History,
    meta: {
      title: '으라차차 감시'
    }
  },
]

const router = new VueRouter({
  mode: 'history',
  base: process.env.BASE_URL,
  routes
})

router.beforeEach((to, from, next) => {
  document.title = to.meta.title || '으라차차!'
  next()
})

export default router
