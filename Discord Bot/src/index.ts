import { Hono } from 'hono';
import { IssueCommentCreatedEvent, IssuesOpenedEvent, PushEvent } from '@octokit/webhooks-types';

const app = new Hono()

app.post('/', async (c) => {
	const data = await c.req.json() as IssueCommentCreatedEvent | IssuesOpenedEvent | PushEvent;
	const headers = c.req.header();

	const githubHeaders = Object.entries(headers).reduce((acc, [key, val]) => {
		if (!key.toLowerCase().includes('github')) return acc;
		return {
			...acc,
			[key]: val,
		};
	}, {});

    const issue = (data as IssueCommentCreatedEvent | IssuesOpenedEvent).issue;
    const commits = (data as PushEvent).commits;
    const repository = data.repository;

    let channels: Array<keyof typeof channelMap> = [];
    const channelMap = {
		'Acoustics & Comms': '1252507800391778315/tvuUS1Gbg2vlhMvi23WgKJHeItVzz1akIcw7wBwjsIKcEp3pmduoMjPzfy59svHTeAVC',
		'Design Challenges': '1251171741389033485/UlhaEjqUq_O7psqudpq2eD6jo729D6TRAVOHBc1Vk1Qq16Fg-CcloelZWJW9r9_jj277',
		'General': '1251172177554968606/qPj4AtLGIwHNMAwVlO0yBPFO15QSd6zlLbmVhDRPHKFqeTLVNjTx1aUEVuCH_59Q8Y1Q',
		'GUI & Software': '1251166757473746995/Saan5Tp6n5_FGPKjUIlIZMA301gDeEpyaNkGB8k1ByerzYy4Wxgt6jOTanc1kfPFHxe8',
		'Hull & Frame': '1251171541362933860/s7_DaYMOB_glUL1U-dKGjSIkxhKT9-RAE860gTRREbqtvt8Uz89a6c4FLd6hvWS7DwQe',
		'Localization': '1252508582084214825/BmZdSPmb9FBV4loMWlDI-hF_LzkuVE51QUfMTKD6Q1iNOUhKSnKlXn0rJe2-dwz7R-th',
		'Manipulators': '1252508721683365938/Ca_vH4H7K2-ktlOHzhpKu6JdXxBH06cvLCyjaHIAGg9Pjojo6ludhT_p82_FTtJtPcOH',
		'Mission Control': '1252508848208478258/7XJGQ_JSNtyMPD_AjhiZXjrDXhslVotBoaYgW0ovDzEhXcCPW88wj6HiM5JryICp7U4n',
		'Object Detection': '1252508973806911539/vbwt2t1jEC7seAHbPTb_dxE6-lewUDl7Z7jSnvSguZGhJR-DuP_RXXV9Vt_77jZdNnsy',
		'Power Management': '1252509066798960703/3WXn73PTXEpWfWbxF2EOne4NyTbpH6BC_86Vxscoaf33B90YLt39DmhNjWNE8VBplyPA',
		'Propulsion': '1252509180829634680/ethHQe_3Khdwo0ON2YrVAimEfSTqTotJv-I1z5_ZgRikUKazd3NzBj5F8yYakeF_6YCl',

		'general': '1251172177554968606/qPj4AtLGIwHNMAwVlO0yBPFO15QSd6zlLbmVhDRPHKFqeTLVNjTx1aUEVuCH_59Q8Y1Q',
    };

    //  If the event is a new issue or comment on an issue
    if (issue) channels = (issue.labels || [ { name: 'General' } ]).map(({ name }) => name as keyof typeof channelMap);

    //  If the event is a new commit/push
    else if (commits) channels = [ repository.name.replace('2025-', '') as keyof typeof channelMap ];

    //  Loop through each channel and send the update
    const promises: Array<Promise<any>> = [];
    channels.forEach((name) => {
		promises.push(fetch(`https://discord.com/api/webhooks/${channelMap[name]}/github`, {
			method: 'POST',
			body: JSON.stringify(data),
			headers: {
				'Content-Type': 'application/json',
				'accept': 'application/json',
				...githubHeaders
			},
		}));
    });

	return c.json(await Promise.all(promises));
});

export default app
