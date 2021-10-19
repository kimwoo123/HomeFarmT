const { Schedule } = require("../../models/index")
const { User } = require("../../models/index")
const { GraphQLScalarType, Kind } = require('graphql');

module.exports = {
  Query: {
    allSchedule: async () =>{
      if(context.user) {
        console.log(context.user)
      }
      const getSchedules = await Schedule.findAll();
      return getSchedules;
    },
    getSchedule: async (_, args, context) => {
      if (!context) {
        throw new Error('로그인이 필요합니다')
      }
      const user = await User.findOne({ where: { email: context.hashedEmail }})
      const userid = user.dataValues.userid
      const userSchedule = await Schedule.findAll({ where: { userid: userid }});
      let scheduleList = []
      for (let schedule of userSchedule) {
        scheduleList.push(schedule.dataValues)
      }
      return scheduleList
    }
  },
  Mutation: {
    createSchedule: async (_, { schedule_time, schedule_title, schedule_desc, schedule_status }, context) => {
      if (!context) {
        throw new Error('인증된 유저가 아닙니다')
      }
      const user = await User.findOne({ where: { email: context.hashedEmail }})
      const userid = user.dataValues.userid
      const newSchedule = await Schedule.create({ 
        schedule_time: schedule_time,
        schedule_title: schedule_title,
        schedule_desc: schedule_desc,
        schedule_status: schedule_status,
        userid: userid
       })
      return newSchedule
    },
    updateScheduleStatus: async (_, { scheduleid, schedule_status }) => {
      await Schedule.update({ schedule_status: schedule_status }, {
        where: {
          scheduleid: scheduleid
        }
      })
      const result = {
        scheduleid: scheduleid,
        schedule_status: schedule_status
      }
      return result
    },
    deleteSchedule: async (_, { scheduleid }) => {
      await Schedule.destroy({ where: { scheduleid }})
    }
  }
};
